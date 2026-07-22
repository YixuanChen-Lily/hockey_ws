#!/usr/bin/env python3

import math
import time
from enum import Enum, auto
from threading import Lock
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hockey_interfaces.action import Spin


class SpinState(Enum):
    WAIT_FOR_POSE = auto()
    SPIN = auto()
    DONE = auto()


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class SpinServer(Node):
    """Action server that spins one robot in place."""

    def __init__(self) -> None:
        super().__init__("spin_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("action_name", "spin")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("pose_timeout_sec", 1.0)

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
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)

        self._pose_lock = Lock()
        self._latest_yaw: Optional[float] = None
        self._latest_pose_time = None
        self._goal_lock = Lock()
        self._goal_active = False
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
            Spin,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Spin action server ready:\n"
            f"  robot_id = {self.robot_id}\n"
            f"  pose     = {self.pose_topic}\n"
            f"  cmd_vel  = {self.cmd_vel_topic}\n"
            f"  action   = {self.action_name}"
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        with self._pose_lock:
            self._latest_yaw = yaw_from_quaternion(message.pose.orientation)
            self._latest_pose_time = self.get_clock().now()

    def _goal_callback(self, request: Spin.Goal) -> GoalResponse:
        if request.rotations <= 0:
            self.get_logger().warning("Rejected goal: rotations must be positive.")
            return GoalResponse.REJECT
        if request.angular_speed <= 0.0 or not math.isfinite(request.angular_speed):
            self.get_logger().warning("Rejected goal: angular_speed must be positive.")
            return GoalResponse.REJECT
        if request.timeout_sec <= 0.0 or not math.isfinite(request.timeout_sec):
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

    def _execute_callback(self, goal_handle) -> Spin.Result:
        request = goal_handle.request
        result = Spin.Result()
        feedback = Spin.Feedback()
        state = SpinState.WAIT_FOR_POSE
        target_rotation = 2.0 * math.pi * float(request.rotations)
        accumulated_rotation = 0.0
        previous_yaw = None
        start_time = time.monotonic()
        control_period = 1.0 / max(self.control_rate_hz, 1.0)

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Spin canceled."
                    result.rotation_completed = accumulated_rotation
                    return result

                if time.monotonic() - start_time > request.timeout_sec:
                    self._stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f"Spin timed out after {request.timeout_sec:.1f} seconds."
                    )
                    result.rotation_completed = accumulated_rotation
                    return result

                yaw = self._get_fresh_yaw()
                if yaw is None:
                    self._stop_robot()
                    state = SpinState.WAIT_FOR_POSE
                    self._publish_feedback(
                        goal_handle,
                        feedback,
                        state,
                        rotation_remaining=target_rotation - accumulated_rotation,
                    )
                    time.sleep(control_period)
                    continue

                if state == SpinState.WAIT_FOR_POSE:
                    state = SpinState.SPIN
                    previous_yaw = yaw

                if state == SpinState.SPIN:
                    if previous_yaw is None:
                        previous_yaw = yaw

                    yaw_change = wrap_to_pi(yaw - previous_yaw)
                    accumulated_rotation += max(0.0, yaw_change)
                    previous_yaw = yaw

                    if accumulated_rotation >= target_rotation:
                        self._stop_robot()
                        state = SpinState.DONE
                    else:
                        self._publish_velocity(request.angular_speed)

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    state,
                    rotation_remaining=max(0.0, target_rotation - accumulated_rotation),
                )

                if state == SpinState.DONE:
                    goal_handle.succeed()
                    result.success = True
                    result.message = f"Completed {request.rotations} rotations."
                    result.rotation_completed = accumulated_rotation
                    self.get_logger().info(result.message)
                    return result

                time.sleep(control_period)

            self._stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = "ROS shutdown interrupted spin."
            result.rotation_completed = accumulated_rotation
            return result

        except Exception as exception:
            self._stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = f"Spin exception: {exception}"
            result.rotation_completed = accumulated_rotation
            self.get_logger().error(result.message)
            return result

        finally:
            self._stop_robot()
            with self._goal_lock:
                self._goal_active = False

    def _get_fresh_yaw(self) -> Optional[float]:
        with self._pose_lock:
            yaw = self._latest_yaw
            pose_time = self._latest_pose_time
        if yaw is None or pose_time is None:
            return None
        pose_age = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        if pose_age > self.pose_timeout_sec:
            return None
        return yaw

    def _publish_feedback(
        self,
        goal_handle,
        feedback: Spin.Feedback,
        state: SpinState,
        rotation_remaining: float,
    ) -> None:
        feedback.state = state.name
        feedback.rotation_remaining = float(rotation_remaining)
        goal_handle.publish_feedback(feedback)

    def _publish_velocity(self, angular_z: float) -> None:
        command = Twist()
        command.angular.z = float(angular_z)
        self._cmd_vel_publisher.publish(command)

    def _stop_robot(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def destroy_node(self) -> None:
        self._stop_robot()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpinServer()
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
