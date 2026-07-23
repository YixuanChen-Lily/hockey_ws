#!/usr/bin/env python3

import math
import time
from enum import Enum, auto
from threading import Lock
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hockey_interfaces.action import NavigateToPoint
from hockey_controller.navigation_server import clamp, wrap_to_pi
from hockey_controller.navigation_server import yaw_from_quaternion


class SafeNavigationState(Enum):
    WAIT_FOR_POSE = auto()
    TRACK_GOAL = auto()
    DONE = auto()


class SafeNavigationServer(Node):
    """Action server using a conservative proportional go-to-point controller."""

    def __init__(self) -> None:
        super().__init__("safe_navigation_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("action_name", "safe_navigate_to_point")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("distance_gain", 0.7)
        self.declare_parameter("heading_gain", 1.8)
        self.declare_parameter("lateral_gain", 0.6)
        self.declare_parameter("linearization_angle_limit", 0.6)
        self.declare_parameter("hard_heading_limit", 1.2)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self.pose_topic = (
            pose_topic
            if pose_topic
            else f"/vrpn_mocap/dji_robot_{self.robot_id}/pose"
        )
        self.cmd_vel_topic = (
            cmd_vel_topic if cmd_vel_topic else f"/robot{self.robot_id}/cmd_vel"
        )
        self.action_name = str(self.get_parameter("action_name").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.distance_gain = float(self.get_parameter("distance_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.lateral_gain = float(self.get_parameter("lateral_gain").value)
        self.linearization_angle_limit = float(
            self.get_parameter("linearization_angle_limit").value
        )
        self.hard_heading_limit = float(
            self.get_parameter("hard_heading_limit").value
        )
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel").value
        )

        self._pose_lock = Lock()
        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._latest_pose_time = None
        self._goal_lock = Lock()
        self._goal_active = False
        self._last_linear_velocity = 0.0
        self._last_angular_velocity = 0.0
        self._callback_group = ReentrantCallbackGroup()

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self._pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            NavigateToPoint,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Safe navigation action server ready:\n"
            f"  robot_id = {self.robot_id}\n"
            f"  pose     = {self.pose_topic}\n"
            f"  cmd_vel  = {self.cmd_vel_topic}\n"
            f"  action   = {self.action_name}"
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        with self._pose_lock:
            self._latest_pose = (
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
            self._latest_pose_time = self.get_clock().now()

    def _goal_callback(self, request: NavigateToPoint.Goal) -> GoalResponse:
        values = (
            request.target_x,
            request.target_y,
            request.linear_speed,
            request.angular_speed,
            request.timeout_sec,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning("Rejected goal: invalid number.")
            return GoalResponse.REJECT
        if request.linear_speed <= 0.0 or request.angular_speed <= 0.0:
            self.get_logger().warning("Rejected goal: speeds must be positive.")
            return GoalResponse.REJECT
        if request.timeout_sec <= 0.0:
            self.get_logger().warning("Rejected goal: timeout must be positive.")
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning("Rejected goal: already running.")
                return GoalResponse.REJECT
            self._goal_active = True

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.get_logger().warning("Cancel request accepted.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> NavigateToPoint.Result:
        request = goal_handle.request
        result = NavigateToPoint.Result()
        feedback = NavigateToPoint.Feedback()
        state = SafeNavigationState.WAIT_FOR_POSE
        start_time = time.monotonic()
        control_period = 1.0 / max(self.control_rate_hz, 1.0)
        final_distance = -1.0

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Safe navigation canceled."
                    result.final_distance = final_distance
                    return result

                if time.monotonic() - start_time > request.timeout_sec:
                    self._stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f"Safe navigation timed out after "
                        f"{request.timeout_sec:.1f} seconds."
                    )
                    result.final_distance = final_distance
                    return result

                current_pose = self._get_fresh_pose()
                if current_pose is None:
                    self._stop_robot()
                    state = SafeNavigationState.WAIT_FOR_POSE
                    self._publish_feedback(
                        goal_handle,
                        feedback,
                        state,
                        distance_remaining=-1.0,
                    )
                    time.sleep(control_period)
                    continue

                x, y, yaw = current_pose
                dx = request.target_x - x
                dy = request.target_y - y
                distance = math.hypot(dx, dy)
                final_distance = distance

                if distance <= self.position_tolerance:
                    self._stop_robot()
                    state = SafeNavigationState.DONE
                else:
                    state = SafeNavigationState.TRACK_GOAL
                    linear_velocity, angular_velocity = self._compute_command(
                        dx,
                        dy,
                        yaw,
                        distance,
                        request.linear_speed,
                        request.angular_speed,
                        control_period,
                    )
                    self._publish_velocity(linear_velocity, angular_velocity)

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    state,
                    distance_remaining=distance,
                )

                if state == SafeNavigationState.DONE:
                    goal_handle.succeed()
                    result.success = True
                    result.message = (
                        f"Safely reached "
                        f"({request.target_x:.2f}, {request.target_y:.2f})."
                    )
                    result.final_distance = distance
                    self.get_logger().info(result.message)
                    return result

                time.sleep(control_period)

            self._stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = "ROS shutdown interrupted safe navigation."
            result.final_distance = final_distance
            return result

        except Exception as exception:
            self._stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = f"Safe navigation exception: {exception}"
            result.final_distance = final_distance
            self.get_logger().error(result.message)
            return result

        finally:
            self._stop_robot()
            with self._goal_lock:
                self._goal_active = False

    def _compute_command(
        self,
        dx: float,
        dy: float,
        yaw: float,
        distance: float,
        max_linear_speed: float,
        max_angular_speed: float,
        control_period: float,
    ) -> Tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x_error = cos_yaw * dx + sin_yaw * dy
        body_y_error = -sin_yaw * dx + cos_yaw * dy

        heading_error = wrap_to_pi(math.atan2(body_y_error, body_x_error))
        linearized_heading_error = clamp(
            heading_error,
            -self.linearization_angle_limit,
            self.linearization_angle_limit,
        )

        if abs(heading_error) >= self.hard_heading_limit:
            desired_linear_velocity = 0.0
        else:
            heading_scale = 1.0 - (
                abs(heading_error) / max(self.hard_heading_limit, 1e-6)
            )
            desired_linear_velocity = (
                self.distance_gain
                * max(body_x_error, 0.0)
                * clamp(heading_scale, 0.0, 1.0)
            )

        lateral_angle_error = body_y_error / max(distance, 1e-6)
        desired_angular_velocity = (
            self.heading_gain * linearized_heading_error
            + self.lateral_gain * lateral_angle_error
        )

        desired_linear_velocity = clamp(
            desired_linear_velocity,
            0.0,
            max_linear_speed,
        )
        desired_angular_velocity = clamp(
            desired_angular_velocity,
            -max_angular_speed,
            max_angular_speed,
        )

        linear_velocity = self._limit_rate(
            desired_linear_velocity,
            self._last_linear_velocity,
            self.max_linear_accel,
            control_period,
        )
        angular_velocity = self._limit_rate(
            desired_angular_velocity,
            self._last_angular_velocity,
            self.max_angular_accel,
            control_period,
        )
        self._last_linear_velocity = linear_velocity
        self._last_angular_velocity = angular_velocity

        return linear_velocity, angular_velocity

    def _limit_rate(
        self,
        target: float,
        previous: float,
        max_rate: float,
        control_period: float,
    ) -> float:
        max_delta = abs(max_rate) * control_period
        return previous + clamp(target - previous, -max_delta, max_delta)

    def _get_fresh_pose(self) -> Optional[Tuple[float, float, float]]:
        with self._pose_lock:
            pose = self._latest_pose
            pose_time = self._latest_pose_time
        if pose is None or pose_time is None:
            return None
        pose_age = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        if pose_age > self.pose_timeout_sec:
            return None
        return pose

    def _publish_feedback(
        self,
        goal_handle,
        feedback: NavigateToPoint.Feedback,
        state: SafeNavigationState,
        distance_remaining: float,
    ) -> None:
        feedback.state = state.name
        feedback.distance_remaining = float(distance_remaining)
        goal_handle.publish_feedback(feedback)

    def _publish_velocity(self, linear_x: float, angular_z: float) -> None:
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        self._cmd_vel_publisher.publish(command)

    def _stop_robot(self) -> None:
        self._last_linear_velocity = 0.0
        self._last_angular_velocity = 0.0
        self._cmd_vel_publisher.publish(Twist())

    def destroy_node(self) -> None:
        self._stop_robot()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeNavigationServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
