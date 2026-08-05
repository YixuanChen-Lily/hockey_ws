#!/usr/bin/env python3

import math
import time
from enum import Enum, auto
from threading import Event, Lock
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hockey_interfaces.action import NavigateToPoint
from hockey_interfaces.action import ShootPuck
from hockey_interfaces.action import Spin
from hockey_controller.control_utils import clamp
from hockey_controller.control_utils import wrap_to_pi
from hockey_controller.control_utils import yaw_from_quaternion


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
        self.declare_parameter("pose_timeout_sec", 150.0)
        self.declare_parameter("action_wait_timeout_sec", 150.0)
        self.declare_parameter("align_gain", 2.0)
        self.declare_parameter("align_timeout_sec", 150.0)
        self.declare_parameter("heading_tolerance", 0.08)
        self.declare_parameter("shooting_pose_position_tolerance", 0.04)
        self.declare_parameter("shooting_pose_timeout_sec", 150.0)
        self.declare_parameter("safe_lookahead_distance", 0.25)
        self.declare_parameter("shooting_center_to_puck_distance", -1.0)
        self.declare_parameter("avoid_puck_during_align", True)
        self.declare_parameter("align_puck_angle_margin_deg", 12.0)
        self.declare_parameter("shooting_puck_obstacle_enabled", True)
        self.declare_parameter("shooting_puck_obstacle_radius", 0.10)
        self.declare_parameter("post_hit_wait_sec", 3.0)
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
        self.shooting_pose_position_tolerance = float(
            self.get_parameter("shooting_pose_position_tolerance").value
        )
        self.shooting_pose_timeout_sec = float(
            self.get_parameter("shooting_pose_timeout_sec").value
        )
        self.safe_lookahead_distance = float(
            self.get_parameter("safe_lookahead_distance").value
        )
        self.shooting_center_to_puck_distance = float(
            self.get_parameter("shooting_center_to_puck_distance").value
        )
        self.avoid_puck_during_align = bool(
            self.get_parameter("avoid_puck_during_align").value
        )
        self.align_puck_angle_margin = math.radians(
            float(self.get_parameter("align_puck_angle_margin_deg").value)
        )
        self.shooting_puck_obstacle_enabled = bool(
            self.get_parameter("shooting_puck_obstacle_enabled").value
        )
        self.shooting_puck_obstacle_radius = float(
            self.get_parameter("shooting_puck_obstacle_radius").value
        )
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
        self._safe_param_client = self.create_client(
            SetParametersAtomically,
            "safe_navigation_server/set_parameters_atomically",
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
        if request.role not in ("shooter", "passer", "single"):
            self.get_logger().warning("Rejected shooting goal: invalid role.")
            return GoalResponse.REJECT
        if request.target_radius <= 0.0:
            self.get_logger().warning("Rejected shooting goal: radius must be positive.")
            return GoalResponse.REJECT
        if request.approach_distance < 0.0:
            self.get_logger().warning(
                "Rejected shooting goal: approach_distance must be non-negative."
            )
            return GoalResponse.REJECT
        if request.contact_gap < 0.0:
            self.get_logger().warning(
                "Rejected shooting goal: contact_gap must be non-negative."
            )
            return GoalResponse.REJECT
        if request.spin_direction not in ("ccw", "cw"):
            self.get_logger().warning(
                "Rejected shooting goal: spin_direction must be ccw or cw."
            )
            return GoalResponse.REJECT
        if request.spin_angle_deg < 0.0 or not math.isfinite(request.spin_angle_deg):
            self.get_logger().warning(
                "Rejected shooting goal: spin_angle_deg must be non-negative."
            )
            return GoalResponse.REJECT
        if request.spin_angle_deg <= 0.0 and request.spin_rotations <= 0:
            self.get_logger().warning(
                "Rejected shooting goal: spin_rotations must be positive "
                "when spin_angle_deg is 0."
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

                if time.monotonic() - start_time > request.timeout_sec:
                    self._stop_robot()
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f"Shooting timed out after {request.timeout_sec:.1f} seconds."
                    )
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
                shoot_yaw = math.atan2(
                    target[1] - puck_pose[1],
                    target[0] - puck_pose[0],
                )
                robot_point, align_yaw, spin_speed = self._spin_shot_pose(
                    puck_pose,
                    shoot_yaw,
                    request,
                )

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.APPROACH_PUCK,
                    puck_distance,
                    attempts,
                )
                self._set_puck_obstacle(puck_pose)
                self._safe_navigate(
                    robot_point,
                    request.linear_speed,
                    request.angular_speed,
                    request.timeout_sec,
                )
                self._clear_puck_obstacle()

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.ALIGN_TO_SHOOT,
                    puck_distance,
                    attempts,
                )
                self._align_to_yaw(
                    align_yaw,
                    request.angular_speed,
                    avoid_puck=True,
                )
                self._drive_to_shoot_pose(
                    robot_point,
                    align_yaw,
                    request.linear_speed,
                    request.angular_speed,
                )
                self._ensure_center_puck_reach(
                    align_yaw,
                    request.linear_speed,
                    request.angular_speed,
                    request.contact_gap,
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
                    float(request.spin_angle_deg),
                    spin_speed,
                    request.timeout_sec,
                )

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    ShootingState.WAIT_FOR_PUCK,
                    puck_distance,
                    attempts,
                )
                reached, puck_distance = self._wait_for_puck_target(
                    goal_handle,
                    feedback,
                    request,
                    attempts,
                )
                if reached:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Puck reached shooting target area."
                    result.final_puck_distance = puck_distance
                    result.attempts = attempts
                    return result

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
            self._clear_puck_obstacle()
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
        self._set_safe_nav_parameters(
            {
                "use_target_pose": False,
                "orient_to_target": False,
            }
        )
        goal = NavigateToPoint.Goal()
        goal.target_x = float(point[0])
        goal.target_y = float(point[1])
        goal.linear_speed = float(linear_speed)
        goal.angular_speed = float(angular_speed)
        goal.timeout_sec = float(timeout_sec)
        self._send_goal(self._safe_nav_client, goal)

    def _spin(
        self,
        rotations: int,
        spin_angle_deg: float,
        angular_speed: float,
        timeout_sec: float,
    ) -> None:
        goal = Spin.Goal()
        goal.rotations = int(rotations)
        goal.spin_angle_deg = float(spin_angle_deg)
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

    def _set_puck_obstacle(self, puck_pose: Tuple[float, float, float]) -> None:
        if not self.shooting_puck_obstacle_enabled:
            return
        self.get_logger().info(
            "Setting puck obstacle for safe nav: "
            f"x={puck_pose[0]:.3f}, y={puck_pose[1]:.3f}, "
            f"radius={self.shooting_puck_obstacle_radius:.3f}"
        )
        self._set_safe_nav_parameters(
            {
                "obstacle_x": [puck_pose[0]],
                "obstacle_y": [puck_pose[1]],
                "obstacle_radius": [self.shooting_puck_obstacle_radius],
            }
        )

    def _clear_puck_obstacle(self) -> None:
        if not self.shooting_puck_obstacle_enabled:
            return
        try:
            self._set_safe_nav_parameters(
                {
                    "obstacle_x": [],
                    "obstacle_y": [],
                    "obstacle_radius": [],
                }
            )
        except Exception as exception:
            self.get_logger().warning(f"failed to clear puck obstacle: {exception}")

    def _set_safe_nav_parameters(self, values) -> None:
        if not self._safe_param_client.wait_for_service(
            timeout_sec=self.action_wait_timeout_sec
        ):
            raise RuntimeError("safe_navigation_server parameter service unavailable")
        request = SetParametersAtomically.Request()
        request.parameters = [
            self._parameter_message(name, value)
            for name, value in values.items()
        ]
        future = self._safe_param_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        response = future.result()
        if response is None:
            raise RuntimeError("safe_navigation_server parameter call failed")
        if not response.result.successful:
            raise RuntimeError(
                f"failed to set safe nav parameter: {response.result.reason}"
            )

    def _parameter_message(self, name: str, value) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, str):
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter.value.double_array_value = [float(item) for item in value]
        return parameter

    def _wait_for_puck_target(
        self,
        goal_handle,
        feedback: ShootPuck.Feedback,
        request: ShootPuck.Goal,
        attempts: int,
    ) -> Tuple[bool, float]:
        wait_until = time.monotonic() + max(0.0, self.post_hit_wait_sec)
        puck_distance = self._current_puck_distance(request)
        while rclpy.ok() and time.monotonic() < wait_until:
            if goal_handle.is_cancel_requested:
                return False, puck_distance
            puck_distance = self._current_puck_distance(request)
            self._publish_feedback(
                goal_handle,
                feedback,
                ShootingState.WAIT_FOR_PUCK,
                puck_distance,
                attempts,
            )
            if puck_distance <= request.target_radius:
                return True, puck_distance
            time.sleep(0.05)
        return puck_distance <= request.target_radius, puck_distance

    def _align_to_yaw(
        self,
        target_yaw: float,
        max_angular_speed: float,
        avoid_puck: bool = False,
    ) -> None:
        start_time = time.monotonic()
        control_period = 1.0 / max(self.control_rate_hz, 1.0)
        forced_direction = (
            self._align_direction_away_from_puck(target_yaw)
            if avoid_puck and self.avoid_puck_during_align
            else 0.0
        )
        while rclpy.ok():
            robot_pose = self._fresh_robot_pose()
            if robot_pose is None:
                raise RuntimeError("stale robot pose during shooting align")
            shortest_error = wrap_to_pi(target_yaw - robot_pose[2])
            error = (
                self._directed_yaw_error(target_yaw, robot_pose[2], forced_direction)
                if forced_direction
                else shortest_error
            )
            if abs(error) <= self.heading_tolerance:
                self._stop_robot()
                return
            if time.monotonic() - start_time > self.align_timeout_sec:
                raise RuntimeError(
                    f"shooting align timeout: target={target_yaw:.3f}, "
                    f"current={robot_pose[2]:.3f}, error={shortest_error:.3f}"
                )
            twist = Twist()
            twist.angular.z = clamp(
                self.align_gain * error,
                -abs(max_angular_speed),
                abs(max_angular_speed),
            )
            self._cmd_vel_publisher.publish(twist)
            time.sleep(control_period)

    def _align_direction_away_from_puck(self, target_yaw: float) -> float:
        robot_pose = self._fresh_robot_pose()
        puck_pose = self._fresh_puck_pose()
        if robot_pose is None or puck_pose is None:
            return 0.0

        rx, ry, current_yaw = robot_pose
        puck_dx = puck_pose[0] - rx
        puck_dy = puck_pose[1] - ry
        puck_distance = math.hypot(puck_dx, puck_dy)
        if puck_distance < 1e-6:
            return 0.0

        hit_radius = (
            self._center_to_puck_distance(0.0)
            + self.shooting_puck_obstacle_radius
        )
        if puck_distance > hit_radius:
            return 0.0

        puck_angle = math.atan2(puck_dy, puck_dx)
        ccw_sweep = self._positive_angle_delta(target_yaw, current_yaw)
        cw_sweep = self._positive_angle_delta(current_yaw, target_yaw)
        shortest_direction = 1.0 if ccw_sweep <= cw_sweep else -1.0

        ccw_hits = self._angle_in_ccw_sweep(
            puck_angle,
            current_yaw,
            ccw_sweep,
            self.align_puck_angle_margin,
        )
        cw_hits = self._angle_in_ccw_sweep(
            puck_angle,
            target_yaw,
            cw_sweep,
            self.align_puck_angle_margin,
        )

        if shortest_direction > 0.0 and ccw_hits and not cw_hits:
            self.get_logger().warning(
                "Align CCW sweep may hit puck; rotating CW to target yaw instead."
            )
            return -1.0
        if shortest_direction < 0.0 and cw_hits and not ccw_hits:
            self.get_logger().warning(
                "Align CW sweep may hit puck; rotating CCW to target yaw instead."
            )
            return 1.0
        return 0.0

    def _directed_yaw_error(
        self,
        target_yaw: float,
        current_yaw: float,
        direction: float,
    ) -> float:
        if direction > 0.0:
            return self._positive_angle_delta(target_yaw, current_yaw)
        return -self._positive_angle_delta(current_yaw, target_yaw)

    def _positive_angle_delta(self, target_yaw: float, current_yaw: float) -> float:
        return (target_yaw - current_yaw) % (2.0 * math.pi)

    def _angle_in_ccw_sweep(
        self,
        angle: float,
        start_yaw: float,
        sweep: float,
        margin: float,
    ) -> bool:
        delta = self._positive_angle_delta(angle, start_yaw)
        return delta <= sweep + margin or delta >= 2.0 * math.pi - margin

    def _drive_to_shoot_pose(
        self,
        target_point: Tuple[float, float],
        target_yaw: float,
        max_linear_speed: float,
        max_angular_speed: float,
    ) -> None:
        self._drive_to_point_then_align(
            target_point,
            target_yaw,
            max_linear_speed,
            max_angular_speed,
        )

    def _drive_to_point_then_align(
        self,
        target_point: Tuple[float, float],
        target_yaw: float,
        max_linear_speed: float,
        max_angular_speed: float,
    ) -> None:
        start_time = time.monotonic()
        control_period = 1.0 / max(self.control_rate_hz, 1.0)
        while rclpy.ok():
            robot_pose = self._fresh_robot_pose()
            if robot_pose is None:
                raise RuntimeError("stale robot pose during shooting pose correction")

            dx = target_point[0] - robot_pose[0]
            dy = target_point[1] - robot_pose[1]
            distance = math.hypot(dx, dy)
            if distance <= self.shooting_pose_position_tolerance:
                self._stop_robot()
                self._align_to_yaw(target_yaw, max_angular_speed, avoid_puck=True)
                return
            if time.monotonic() - start_time > self.shooting_pose_timeout_sec:
                yaw_error = wrap_to_pi(target_yaw - robot_pose[2])
                raise RuntimeError(
                    f"shooting pose timeout: distance={distance:.3f}, "
                    f"yaw_error={yaw_error:.3f}"
                )

            heading_error = wrap_to_pi(math.atan2(dy, dx) - robot_pose[2])
            twist = Twist()
            if abs(heading_error) > 0.20:
                twist.angular.z = clamp(
                    2.0 * heading_error,
                    -abs(max_angular_speed),
                    abs(max_angular_speed),
                )
            else:
                twist.linear.x = clamp(
                    1.0 * distance,
                    -abs(max_linear_speed),
                    abs(max_linear_speed),
                )
                twist.angular.z = clamp(
                    1.0 * heading_error,
                    -0.5 * abs(max_angular_speed),
                    0.5 * abs(max_angular_speed),
                )
            self._cmd_vel_publisher.publish(twist)
            time.sleep(control_period)

    def _ensure_center_puck_reach(
        self,
        target_yaw: float,
        max_linear_speed: float,
        max_angular_speed: float,
        contact_gap: float,
    ) -> None:
        robot_pose = self._fresh_robot_pose()
        puck_pose = self._fresh_puck_pose()
        if robot_pose is None or puck_pose is None:
            raise RuntimeError("stale pose during center-puck clearance check")

        max_distance = self._center_to_puck_distance(contact_gap)
        dx = robot_pose[0] - puck_pose[0]
        dy = robot_pose[1] - puck_pose[1]
        distance = math.hypot(dx, dy)
        if distance <= max_distance:
            return

        if distance < 1e-6:
            toward_robot_x = math.cos(target_yaw)
            toward_robot_y = math.sin(target_yaw)
        else:
            toward_robot_x = dx / distance
            toward_robot_y = dy / distance
        corrected_target = (
            puck_pose[0] + max_distance * toward_robot_x,
            puck_pose[1] + max_distance * toward_robot_y,
        )
        self.get_logger().warning(
            "Robot center too far from puck before spin: "
            f"distance={distance:.3f}m, allowed={max_distance:.3f}m. "
            "Moving closer before hit."
        )
        self._drive_to_point_then_align(
            corrected_target,
            target_yaw,
            max_linear_speed,
            max_angular_speed,
        )

    def _target_point(
        self,
        goal_pose: Tuple[float, float, float],
        request: ShootPuck.Goal,
    ) -> Tuple[float, float]:
        if request.role == "shooter":
            return goal_pose[0], goal_pose[1]
        cos_goal = math.cos(goal_pose[2])
        sin_goal = math.sin(goal_pose[2])
        target_x = (
            goal_pose[0]
            + request.offset_x * cos_goal
            - request.offset_y * sin_goal
        )
        target_y = (
            goal_pose[1]
            + request.offset_x * sin_goal
            + request.offset_y * cos_goal
        )
        return target_x, target_y

    def _spin_shot_pose(
        self,
        puck_pose: Tuple[float, float, float],
        shoot_yaw: float,
        request: ShootPuck.Goal,
    ) -> Tuple[Tuple[float, float], float, float]:
        spin_speed = (
            abs(request.angular_speed)
            if request.spin_direction == "ccw"
            else -abs(request.angular_speed)
        )

        if request.spin_direction == "ccw":
            normal_yaw = shoot_yaw - math.pi / 2.0
        else:
            normal_yaw = shoot_yaw + math.pi / 2.0
        side_distance = self._center_to_puck_distance(request.contact_gap)
        robot_point = (
            puck_pose[0] - side_distance * math.cos(normal_yaw),
            puck_pose[1] - side_distance * math.sin(normal_yaw),
        )
        align_yaw = wrap_to_pi(
            shoot_yaw + math.pi + request.shooting_angle_offset
        )
        return robot_point, align_yaw, spin_speed

    def _center_to_puck_distance(self, contact_gap: float) -> float:
        self.shooting_center_to_puck_distance = float(
            self.get_parameter("shooting_center_to_puck_distance").value
        )
        if self.shooting_center_to_puck_distance > 0.0:
            return self.shooting_center_to_puck_distance
        return self.safe_lookahead_distance + max(0.0, contact_gap)

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
