#!/usr/bin/env python3

import math
import time
from enum import Enum, auto
from threading import Event, Lock
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hockey_interfaces.action import NavigateToPoint
from hockey_interfaces.action import ShootPuck
from hockey_interfaces.action import Spin
from hockey_controller.navigation_server import clamp
from hockey_controller.navigation_server import wrap_to_pi
from hockey_controller.navigation_server import yaw_from_quaternion


class ShootingState(Enum):
    WAIT_FOR_POSES = auto()
    CHECK_TARGET = auto()
    APPROACH_PUCK = auto()
    ALIGN_TO_SHOOT = auto()
    HIT_PUCK = auto()
    WAIT_FOR_PUCK = auto()
    DONE = auto()


class ShootingServer(Node):
    """Action server that repeatedly hits the puck toward a target area."""

    def __init__(self) -> None:
        super().__init__("shooting_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("puck_pose_topic", "/vrpn_mocap/puck/pose")
        self.declare_parameter("goal_pose_topic", "/vrpn_mocap/goal/pose")
        self.declare_parameter("action_name", "shoot_puck")
        self.declare_parameter("safe_navigation_action", "safe_navigate_to_point")
        self.declare_parameter("spin_action", "spin")
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("action_wait_timeout_sec", 5.0)
        self.declare_parameter("align_gain", 2.0)
        self.declare_parameter("align_timeout_sec", 5.0)
        self.declare_parameter("heading_tolerance", 0.08)
        self.declare_parameter("post_hit_wait_sec", 1.0)
        self.declare_parameter("control_rate_hz", 20.0)

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
        self.puck_pose_topic = str(self.get_parameter("puck_pose_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.safe_navigation_action = str(
            self.get_parameter("safe_navigation_action").value
        )
        self.spin_action = str(self.get_parameter("spin_action").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.action_wait_timeout_sec = float(
            self.get_parameter("action_wait_timeout_sec").value
        )
        self.align_gain = float(self.get_parameter("align_gain").value)
        self.align_timeout_sec = float(self.get_parameter("align_timeout_sec").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance").value)
        self.post_hit_wait_sec = float(self.get_parameter("post_hit_wait_sec").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self._pose_lock = Lock()
        self._latest_robot_pose: Optional[Tuple[float, float, float]] = None
        self._latest_robot_pose_time = None
        self._latest_puck_pose: Optional[Tuple[float, float, float]] = None
        self._latest_puck_pose_time = None
        self._latest_goal_pose: Optional[Tuple[float, float, float]] = None
        self._latest_goal_pose_time = None
        self._goal_lock = Lock()
        self._goal_active = False
        self._active_goal = None
        self._callback_group = ReentrantCallbackGroup()

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._robot_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseStamped,
            self.puck_pose_topic,
            self._puck_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self._goal_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

        self._safe_nav_client = ActionClient(
            self,
            NavigateToPoint,
            self.safe_navigation_action,
            callback_group=self._callback_group,
        )
        self._spin_client = ActionClient(
            self,
            Spin,
            self.spin_action,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            ShootPuck,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Shooting action server ready:\n"
            f"  robot_id     = {self.robot_id}\n"
            f"  robot pose   = {self.pose_topic}\n"
            f"  puck pose    = {self.puck_pose_topic}\n"
            f"  goal pose    = {self.goal_pose_topic}\n"
            f"  cmd_vel      = {self.cmd_vel_topic}\n"
            f"  action       = {self.action_name}"
        )

    def _goal_callback(self, request: ShootPuck.Goal) -> GoalResponse:
        if request.role not in ("shooter", "passer"):
            self.get_logger().warning("Rejected shooting goal: invalid role.")
            return GoalResponse.REJECT
        if request.target_radius <= 0.0:
            self.get_logger().warning("Rejected shooting goal: radius must be positive.")
            return GoalResponse.REJECT
        if request.approach_distance <= 0.0:
            self.get_logger().warning(
                "Rejected shooting goal: approach_distance must be positive."
            )
            return GoalResponse.REJECT
        if request.timeout_sec <= 0.0 or request.max_attempts <= 0:
            self.get_logger().warning(
                "Rejected shooting goal: timeout and max_attempts must be positive."
            )
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning("Rejected shooting goal: already running.")
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.get_logger().warning("Shooting cancel request accepted.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> ShootPuck.Result:
        request = goal_handle.request
        result = ShootPuck.Result()
        feedback = ShootPuck.Feedback()
        attempts = 0
        start_time = time.monotonic()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_robot()
                    self._cancel_active_child_goal()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Shooting canceled."
                    result.final_puck_distance = self._current_puck_distance(request)
                    result.attempts = attempts
                    return result

                if time.monotonic() - start_time > request.timeout_sec:
                    self._stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f"Shooting timed out after {request.timeout_sec:.1f} seconds."
                    )
                    result.final_puck_distance = self._current_puck_distance(request)
                    result.attempts = attempts
                    return result

                puck_pose = self._fresh_puck_pose()
                goal_pose = self._fresh_goal_pose()
                if puck_pose is None or goal_pose is None:
                    self._publish_feedback(
                        goal_handle,
                        feedback,
                        ShootingState.WAIT_FOR_POSES,
                        float("inf"),
                        attempts,
                    )
                    time.sleep(0.05)
                    continue

                target = self._target_point(goal_pose, request)
                puck_distance = self._distance(puck_pose, target)
                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.CHECK_TARGET,
                    puck_distance,
                    attempts,
                )
                if puck_distance <= request.target_radius:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Puck reached shooting target area."
                    result.final_puck_distance = puck_distance
                    result.attempts = attempts
                    return result

                if attempts >= request.max_attempts:
                    goal_handle.abort()
                    result.success = False
                    result.message = "Shooting failed: max attempts reached."
                    result.final_puck_distance = puck_distance
                    result.attempts = attempts
                    return result

                attempts += 1
                shoot_yaw = math.atan2(target[1] - puck_pose[1], target[0] - puck_pose[0])
                approach_point = (
                    puck_pose[0] - request.approach_distance * math.cos(shoot_yaw),
                    puck_pose[1] - request.approach_distance * math.sin(shoot_yaw),
                )

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.APPROACH_PUCK,
                    puck_distance,
                    attempts,
                )
                self._safe_navigate(
                    approach_point,
                    request.linear_speed,
                    request.angular_speed,
                    request.timeout_sec,
                )

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.ALIGN_TO_SHOOT,
                    puck_distance,
                    attempts,
                )
                self._align_to_yaw(
                    shoot_yaw + request.shooting_angle_offset,
                    request.angular_speed,
                )

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.HIT_PUCK,
                    puck_distance,
                    attempts,
                )
                self._spin(
                    int(request.spin_rotations),
                    request.angular_speed,
                    request.timeout_sec,
                )

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.WAIT_FOR_PUCK,
                    puck_distance,
                    attempts,
                )
                time.sleep(max(0.0, self.post_hit_wait_sec))

            self._stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = "ROS shutdown interrupted shooting."
            result.final_puck_distance = self._current_puck_distance(request)
            result.attempts = attempts
            return result

        except Exception as exception:
            self._stop_robot()
            goal_handle.abort()
            result.success = False
            result.message = f"Shooting exception: {exception}"
            result.final_puck_distance = self._current_puck_distance(request)
            result.attempts = attempts
            self.get_logger().error(result.message)
            return result

        finally:
            self._stop_robot()
            self._active_goal = None
            with self._goal_lock:
                self._goal_active = False

    def _safe_navigate(
        self,
        point: Tuple[float, float],
        linear_speed: float,
        angular_speed: float,
        timeout_sec: float,
    ) -> None:
        goal = NavigateToPoint.Goal()
        goal.target_x = float(point[0])
        goal.target_y = float(point[1])
        goal.linear_speed = float(linear_speed)
        goal.angular_speed = float(angular_speed)
        goal.timeout_sec = float(timeout_sec)
        self._send_goal(self._safe_nav_client, goal)

    def _spin(self, rotations: int, angular_speed: float, timeout_sec: float) -> None:
        goal = Spin.Goal()
        goal.rotations = int(rotations)
        goal.angular_speed = float(angular_speed)
        goal.timeout_sec = float(timeout_sec)
        self._send_goal(self._spin_client, goal)

    def _send_goal(self, client, goal) -> None:
        if not client.wait_for_server(timeout_sec=self.action_wait_timeout_sec):
            raise RuntimeError("child action server unavailable")

        done = Event()
        outcome = {"success": False, "message": "No result"}

        def goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                outcome["message"] = "Child goal rejected"
                done.set()
                return
            self._active_goal = goal_handle
            goal_handle.get_result_async().add_done_callback(goal_result)

        def goal_result(future):
            action_result = future.result().result
            outcome["success"] = bool(action_result.success)
            outcome["message"] = str(action_result.message)
            self._active_goal = None
            done.set()

        client.send_goal_async(goal).add_done_callback(goal_response)
        while rclpy.ok() and not done.wait(0.05):
            pass
        if not outcome["success"]:
            raise RuntimeError(outcome["message"])

    def _align_to_yaw(self, target_yaw: float, max_angular_speed: float) -> None:
        start_time = time.monotonic()
        control_period = 1.0 / max(self.control_rate_hz, 1.0)
        while rclpy.ok():
            robot_pose = self._fresh_robot_pose()
            if robot_pose is None:
                raise RuntimeError("stale robot pose during shooting align")
            error = wrap_to_pi(target_yaw - robot_pose[2])
            if abs(error) <= self.heading_tolerance:
                self._stop_robot()
                return
            if time.monotonic() - start_time > self.align_timeout_sec:
                raise RuntimeError(
                    f"shooting align timeout: target={target_yaw:.3f}, "
                    f"current={robot_pose[2]:.3f}, error={error:.3f}"
                )
            twist = Twist()
            twist.angular.z = clamp(
                self.align_gain * error,
                -abs(max_angular_speed),
                abs(max_angular_speed),
            )
            self._cmd_vel_publisher.publish(twist)
            time.sleep(control_period)

    def _target_point(
        self,
        goal_pose: Tuple[float, float, float],
        request: ShootPuck.Goal,
    ) -> Tuple[float, float]:
        if request.role == "shooter":
            return goal_pose[0], goal_pose[1]
        return goal_pose[0] + request.offset_x, goal_pose[1] + request.offset_y

    def _current_puck_distance(self, request: ShootPuck.Goal) -> float:
        puck_pose = self._fresh_puck_pose()
        goal_pose = self._fresh_goal_pose()
        if puck_pose is None or goal_pose is None:
            return float("inf")
        return self._distance(puck_pose, self._target_point(goal_pose, request))

    def _fresh_robot_pose(self):
        with self._pose_lock:
            pose = self._latest_robot_pose
            pose_time = self._latest_robot_pose_time
        return self._fresh_pose(pose, pose_time)

    def _fresh_puck_pose(self):
        with self._pose_lock:
            pose = self._latest_puck_pose
            pose_time = self._latest_puck_pose_time
        return self._fresh_pose(pose, pose_time)

    def _fresh_goal_pose(self):
        with self._pose_lock:
            pose = self._latest_goal_pose
            pose_time = self._latest_goal_pose_time
        return self._fresh_pose(pose, pose_time)

    def _fresh_pose(self, pose, pose_time):
        if pose is None or pose_time is None:
            return None
        age = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        return pose if age <= self.pose_timeout_sec else None

    def _robot_pose_callback(self, message: PoseStamped) -> None:
        with self._pose_lock:
            self._latest_robot_pose = self._pose_from_message(message)
            self._latest_robot_pose_time = self.get_clock().now()

    def _puck_pose_callback(self, message: PoseStamped) -> None:
        with self._pose_lock:
            self._latest_puck_pose = self._pose_from_message(message)
            self._latest_puck_pose_time = self.get_clock().now()

    def _goal_pose_callback(self, message: PoseStamped) -> None:
        with self._pose_lock:
            self._latest_goal_pose = self._pose_from_message(message)
            self._latest_goal_pose_time = self.get_clock().now()

    def _pose_from_message(self, message: PoseStamped) -> Tuple[float, float, float]:
        pose = message.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    def _publish_feedback(
        self,
        goal_handle,
        feedback: ShootPuck.Feedback,
        state: ShootingState,
        puck_distance: float,
        attempts: int,
    ) -> None:
        feedback.state = state.name
        feedback.puck_distance_to_target = float(puck_distance)
        feedback.attempts = int(attempts)
        goal_handle.publish_feedback(feedback)

    def _cancel_active_child_goal(self) -> None:
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()

    def _stop_robot(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _distance(
        self,
        first: Tuple[float, float, float],
        second: Tuple[float, float],
    ) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def destroy_node(self) -> None:
        self._stop_robot()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShootingServer()
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
