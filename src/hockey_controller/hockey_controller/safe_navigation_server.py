#!/usr/bin/env python3

import math
import ast
import time
from dataclasses import dataclass
from enum import Enum, auto
from threading import Lock
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from hockey_interfaces.action import NavigateToPoint
from hockey_controller.clf_cbf_qp import CircularObstacle
from hockey_controller.clf_cbf_qp import DynamicCircularObstacle
from hockey_controller.clf_cbf_qp import QpResult
from hockey_controller.clf_cbf_qp import compute_nominal_point_velocity
from hockey_controller.clf_cbf_qp import obstacle_arrays_valid
from hockey_controller.clf_cbf_qp import solve_clf_cbf_qp
from hockey_controller.navigation_server import clamp
from hockey_controller.navigation_server import wrap_to_pi
from hockey_controller.navigation_server import yaw_from_quaternion


class SafeNavigationState(Enum):
    WAIT_FOR_POSE = auto()
    TRACK_GOAL = auto()
    ORIENT = auto()
    DONE = auto()


@dataclass
class DynamicRobotState:
    x: float
    y: float
    yaw: float
    timestamp: float
    previous_x: Optional[float] = None
    previous_y: Optional[float] = None
    previous_timestamp: Optional[float] = None
    velocity_x: float = 0.0
    velocity_y: float = 0.0


class SafeNavigationServer(Node):
    """Action server using approximate linearization for a unicycle robot."""

    def __init__(self) -> None:
        super().__init__("safe_navigation_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("action_name", "safe_navigate_to_point")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("heading_tolerance", 0.08)
        self.declare_parameter("pose_timeout_sec", 150.0)
        self.declare_parameter("target_pose_timeout_sec", 150.0)
        self.declare_parameter("lookahead_distance", 0.25)
        self.declare_parameter("point_gain", 0.8)
        self.declare_parameter("clf_gain", 1.0)
        self.declare_parameter("cbf_gain", 2.0)
        self.declare_parameter("slack_weight", 100.0)
        self.declare_parameter("max_point_speed", 0.4)
        self.declare_parameter("obstacles_enabled", True)
        self.declare_parameter("obstacle_safe_margin", 0.10)
        self.declare_parameter("robot_safety_radius", 0.20)
        obstacle_array_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("obstacle_x", [], obstacle_array_descriptor)
        self.declare_parameter("obstacle_y", [], obstacle_array_descriptor)
        self.declare_parameter(
            "obstacle_radius",
            [],
            obstacle_array_descriptor,
        )
        self.declare_parameter("qp_solver", "cvxopt")
        self.declare_parameter("qp_verbose", False)
        self.declare_parameter("diagnostic_log_period_sec", 1.0)
        self.declare_parameter("orient_gain", 2.0)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.0)
        self.declare_parameter("target_offset_x", 0.0)
        self.declare_parameter("target_offset_y", 0.0)
        self.declare_parameter("target_orientation_offset", 0.0)
        self.declare_parameter("orient_to_target", False)
        self.declare_parameter("use_target_pose", True)
        dynamic_ids_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("dynamic_robot_ids", [], dynamic_ids_descriptor)
        self.declare_parameter("dynamic_obstacles_required", True)
        self.declare_parameter("dynamic_obstacle_timeout_sec", 150.0)
        self.declare_parameter("dynamic_obstacle_velocity_filter_gain", 0.5)
        self.declare_parameter("dynamic_obstacle_max_speed", 2.0)
        self.declare_parameter("dynamic_controlled_robot_radius", 0.18)
        self.declare_parameter("dynamic_robot_radius", 0.18)
        self.declare_parameter("dynamic_robot_safety_margin", 0.10)
        self.declare_parameter("dynamic_cbf_gain", 2.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)

        self.pose_topic = (
            pose_topic
            if pose_topic
            else f"/vrpn_mocap/dji_robot_{self.robot_id}/pose"
        )
        self.cmd_vel_topic = (
            cmd_vel_topic if cmd_vel_topic else f"/robot{self.robot_id}/cmd_vel"
        )
        self.target_pose_topic = (
            target_pose_topic
            if target_pose_topic
            else "/vrpn_mocap/hockey_sticks_1/pose"
        )
        self.action_name = str(self.get_parameter("action_name").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.heading_tolerance = float(
            self.get_parameter("heading_tolerance").value
        )
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.target_pose_timeout_sec = float(
            self.get_parameter("target_pose_timeout_sec").value
        )
        self.lookahead_distance = max(
            float(self.get_parameter("lookahead_distance").value),
            1e-3,
        )
        self.point_gain = float(self.get_parameter("point_gain").value)
        self.clf_gain = float(self.get_parameter("clf_gain").value)
        self.cbf_gain = float(self.get_parameter("cbf_gain").value)
        self.slack_weight = float(self.get_parameter("slack_weight").value)
        self.max_point_speed = float(self.get_parameter("max_point_speed").value)
        self.obstacles_enabled = bool(
            self.get_parameter("obstacles_enabled").value
        )
        self.obstacle_safe_margin = float(
            self.get_parameter("obstacle_safe_margin").value
        )
        self.robot_safety_radius = float(
            self.get_parameter("robot_safety_radius").value
        )
        self.obstacle_x = self._parameter_array("obstacle_x")
        self.obstacle_y = self._parameter_array("obstacle_y")
        self.obstacle_radius = self._parameter_array("obstacle_radius")
        self._validate_initial_parameters()
        self.qp_solver = str(self.get_parameter("qp_solver").value)
        self.qp_verbose = bool(self.get_parameter("qp_verbose").value)
        self.diagnostic_log_period_sec = float(
            self.get_parameter("diagnostic_log_period_sec").value
        )
        self.orient_gain = float(self.get_parameter("orient_gain").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel").value
        )
        self.target_offset_x = float(self.get_parameter("target_offset_x").value)
        self.target_offset_y = float(self.get_parameter("target_offset_y").value)
        self.target_orientation_offset = float(
            self.get_parameter("target_orientation_offset").value
        )
        self.orient_to_target = bool(self.get_parameter("orient_to_target").value)
        self.use_target_pose = bool(self.get_parameter("use_target_pose").value)
        self.dynamic_robot_ids = self._sanitize_dynamic_robot_ids(
            self._parameter_int_array("dynamic_robot_ids")
        )
        self.dynamic_obstacles_required = bool(
            self.get_parameter("dynamic_obstacles_required").value
        )
        self.dynamic_obstacle_timeout_sec = float(
            self.get_parameter("dynamic_obstacle_timeout_sec").value
        )
        self.dynamic_obstacle_velocity_filter_gain = float(
            self.get_parameter("dynamic_obstacle_velocity_filter_gain").value
        )
        self.dynamic_obstacle_max_speed = float(
            self.get_parameter("dynamic_obstacle_max_speed").value
        )
        self.dynamic_controlled_robot_radius = float(
            self.get_parameter("dynamic_controlled_robot_radius").value
        )
        self.dynamic_robot_radius = float(
            self.get_parameter("dynamic_robot_radius").value
        )
        self.dynamic_robot_safety_margin = float(
            self.get_parameter("dynamic_robot_safety_margin").value
        )
        self.dynamic_cbf_gain = float(
            self.get_parameter("dynamic_cbf_gain").value
        )
        self._validate_dynamic_obstacle_parameters()

        self._pose_lock = Lock()
        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._latest_pose_time = None
        self._target_pose_lock = Lock()
        self._latest_target_pose: Optional[Tuple[float, float, float]] = None
        self._latest_target_pose_time = None
        self._dynamic_obstacle_lock = Lock()
        self._dynamic_robot_states: Dict[int, DynamicRobotState] = {}
        self._dynamic_robot_subscriptions = {}
        self._goal_lock = Lock()
        self._goal_active = False
        self._last_linear_velocity = 0.0
        self._last_angular_velocity = 0.0
        self._last_qp_warning_time = 0.0
        self._last_diagnostic_log_time = 0.0
        self._last_qp_solve_time_sec = 0.0
        self._loop_count = 0
        self._loop_rate_window_start = time.monotonic()
        self._measured_control_rate_hz = 0.0
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
        self._target_pose_subscription = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._reset_dynamic_robot_subscriptions(self.dynamic_robot_ids)
        self._action_server = ActionServer(
            self,
            NavigateToPoint,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._handle_parameter_update)

        self.get_logger().info(
            "Safe navigation action server ready:\n"
            f"  robot_id    = {self.robot_id}\n"
            f"  pose        = {self.pose_topic}\n"
            f"  target_pose = {self.target_pose_topic}\n"
            f"  cmd_vel     = {self.cmd_vel_topic}\n"
            f"  action      = {self.action_name}\n"
            f"  offset      = ({self.target_offset_x}, {self.target_offset_y})\n"
            f"  orient      = {self.orient_to_target}\n"
            f"  use target pose = {self.use_target_pose}\n"
            f"  obstacles enabled = {self.obstacles_enabled}\n"
            f"  obstacles   = {len(self.obstacle_x)}\n"
            f"  dynamic robots = {self.dynamic_robot_ids}"
        )

    def _handle_parameter_update(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "lookahead_distance":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="lookahead_distance must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="lookahead_distance must be positive",
                    )

            if parameter.name == "point_gain":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="point_gain must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="point_gain must be positive",
                    )

            if parameter.name == "orient_gain":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="orient_gain must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="orient_gain must be positive",
                    )

            if parameter.name == "heading_tolerance":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="heading_tolerance must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="heading_tolerance must be positive",
                    )

            if parameter.name == "target_pose_timeout_sec":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="target_pose_timeout_sec must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="target_pose_timeout_sec must be positive",
                    )

            if parameter.name == "target_pose_topic":
                if parameter.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason="target_pose_topic must be a string",
                    )

            if parameter.name in (
                "clf_gain",
                "cbf_gain",
                "dynamic_cbf_gain",
                "slack_weight",
                "max_point_speed",
            ):
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be positive",
                    )

            if parameter.name == "robot_safety_radius":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="robot_safety_radius must be a float",
                    )
                if parameter.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="robot_safety_radius must be non-negative",
                    )

            if parameter.name == "obstacle_safe_margin":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="obstacle_safe_margin must be a float",
                    )
                if parameter.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="obstacle_safe_margin must be non-negative",
                    )

            if parameter.name == "diagnostic_log_period_sec":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="diagnostic_log_period_sec must be a float",
                    )
                if parameter.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="diagnostic_log_period_sec must be non-negative",
                    )

            if parameter.name == "dynamic_obstacle_timeout_sec":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="dynamic_obstacle_timeout_sec must be a float",
                    )
                if parameter.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="dynamic_obstacle_timeout_sec must be positive",
                    )

            if parameter.name == "dynamic_obstacle_velocity_filter_gain":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "dynamic_obstacle_velocity_filter_gain must be a float"
                        ),
                    )
                if parameter.value < 0.0 or parameter.value > 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "dynamic_obstacle_velocity_filter_gain must be in [0, 1]"
                        ),
                    )

            if parameter.name in (
                "dynamic_obstacle_max_speed",
                "dynamic_controlled_robot_radius",
                "dynamic_robot_radius",
                "dynamic_robot_safety_margin",
            ):
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be a float",
                    )
                if parameter.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be non-negative",
                    )

            if parameter.name == "qp_solver":
                if parameter.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason="qp_solver must be a string",
                    )
                if not parameter.value:
                    return SetParametersResult(
                        successful=False,
                        reason="qp_solver must name a qpsolvers backend",
                    )

        updated_obstacle_x = self.obstacle_x
        updated_obstacle_y = self.obstacle_y
        updated_obstacle_radius = self.obstacle_radius
        for parameter in parameters:
            try:
                if parameter.name == "obstacle_x":
                    updated_obstacle_x = self._coerce_float_array(parameter.value)
                elif parameter.name == "obstacle_y":
                    updated_obstacle_y = self._coerce_float_array(parameter.value)
                elif parameter.name == "obstacle_radius":
                    updated_obstacle_radius = self._coerce_float_array(
                        parameter.value
                    )
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be an array of numbers",
                )

        updated_dynamic_robot_ids = self.dynamic_robot_ids
        for parameter in parameters:
            try:
                if parameter.name == "dynamic_robot_ids":
                    updated_dynamic_robot_ids = self._sanitize_dynamic_robot_ids(
                        self._coerce_int_array(parameter.value)
                    )
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason="dynamic_robot_ids must be an array of integers",
                )

        if not self._obstacle_arrays_can_update(
            updated_obstacle_x,
            updated_obstacle_y,
            updated_obstacle_radius,
        ):
            return SetParametersResult(
                successful=False,
                reason=(
                    "obstacle_x, obstacle_y, and obstacle_radius must have "
                    "equal length when all are set, and non-negative radii"
                ),
            )

        for parameter in parameters:
            if parameter.name == "lookahead_distance":
                self.lookahead_distance = float(parameter.value)
            elif parameter.name == "point_gain":
                self.point_gain = float(parameter.value)
            elif parameter.name == "clf_gain":
                self.clf_gain = float(parameter.value)
            elif parameter.name == "cbf_gain":
                self.cbf_gain = float(parameter.value)
            elif parameter.name == "slack_weight":
                self.slack_weight = float(parameter.value)
            elif parameter.name == "max_point_speed":
                self.max_point_speed = float(parameter.value)
            elif parameter.name == "obstacles_enabled":
                self.obstacles_enabled = bool(parameter.value)
            elif parameter.name == "obstacle_safe_margin":
                self.obstacle_safe_margin = float(parameter.value)
            elif parameter.name == "robot_safety_radius":
                self.robot_safety_radius = float(parameter.value)
            elif parameter.name == "obstacle_x":
                self.obstacle_x = updated_obstacle_x
            elif parameter.name == "obstacle_y":
                self.obstacle_y = updated_obstacle_y
            elif parameter.name == "obstacle_radius":
                self.obstacle_radius = updated_obstacle_radius
            elif parameter.name == "qp_solver":
                self.qp_solver = str(parameter.value)
            elif parameter.name == "qp_verbose":
                self.qp_verbose = bool(parameter.value)
            elif parameter.name == "diagnostic_log_period_sec":
                self.diagnostic_log_period_sec = float(parameter.value)
            elif parameter.name == "orient_gain":
                self.orient_gain = float(parameter.value)
            elif parameter.name == "heading_tolerance":
                self.heading_tolerance = float(parameter.value)
            elif parameter.name == "target_pose_timeout_sec":
                self.target_pose_timeout_sec = float(parameter.value)
            elif parameter.name == "target_pose_topic":
                new_topic = str(parameter.value)
                if not new_topic:
                    new_topic = "/vrpn_mocap/hockey_sticks_1/pose"
                self._reset_target_pose_subscription(new_topic)
            elif parameter.name == "target_offset_x":
                self.target_offset_x = float(parameter.value)
            elif parameter.name == "target_offset_y":
                self.target_offset_y = float(parameter.value)
            elif parameter.name == "target_orientation_offset":
                self.target_orientation_offset = float(parameter.value)
            elif parameter.name == "orient_to_target":
                self.orient_to_target = bool(parameter.value)
            elif parameter.name == "use_target_pose":
                self.use_target_pose = bool(parameter.value)
            elif parameter.name == "dynamic_robot_ids":
                self.dynamic_robot_ids = updated_dynamic_robot_ids
                self._reset_dynamic_robot_subscriptions(self.dynamic_robot_ids)
            elif parameter.name == "dynamic_obstacles_required":
                self.dynamic_obstacles_required = bool(parameter.value)
            elif parameter.name == "dynamic_obstacle_timeout_sec":
                self.dynamic_obstacle_timeout_sec = float(parameter.value)
            elif parameter.name == "dynamic_obstacle_velocity_filter_gain":
                self.dynamic_obstacle_velocity_filter_gain = float(parameter.value)
            elif parameter.name == "dynamic_obstacle_max_speed":
                self.dynamic_obstacle_max_speed = float(parameter.value)
            elif parameter.name == "dynamic_controlled_robot_radius":
                self.dynamic_controlled_robot_radius = float(parameter.value)
            elif parameter.name == "dynamic_robot_radius":
                self.dynamic_robot_radius = float(parameter.value)
            elif parameter.name == "dynamic_robot_safety_margin":
                self.dynamic_robot_safety_margin = float(parameter.value)
            elif parameter.name == "dynamic_cbf_gain":
                self.dynamic_cbf_gain = float(parameter.value)

        return SetParametersResult(successful=True)

    def _reset_target_pose_subscription(self, topic: str) -> None:
        if topic == self.target_pose_topic:
            return
        self.destroy_subscription(self._target_pose_subscription)
        with self._target_pose_lock:
            self._latest_target_pose = None
            self._latest_target_pose_time = None
        self.target_pose_topic = topic
        self._target_pose_subscription = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"Updated target pose subscription: {self.target_pose_topic}"
        )

    def _validate_initial_parameters(self) -> None:
        positive_values = {
            "lookahead_distance": self.lookahead_distance,
            "point_gain": self.point_gain,
            "clf_gain": self.clf_gain,
            "cbf_gain": self.cbf_gain,
            "slack_weight": self.slack_weight,
            "max_point_speed": self.max_point_speed,
            "heading_tolerance": self.heading_tolerance,
            "target_pose_timeout_sec": self.target_pose_timeout_sec,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.robot_safety_radius < 0.0:
            raise ValueError("robot_safety_radius must be non-negative")
        if self.obstacle_safe_margin < 0.0:
            raise ValueError("obstacle_safe_margin must be non-negative")
        if not self._obstacle_arrays_valid(
            self.obstacle_x,
            self.obstacle_y,
            self.obstacle_radius,
        ):
            raise ValueError(
                "obstacle_x, obstacle_y, and obstacle_radius must have "
                "equal length and non-negative radii"
            )

    def _validate_dynamic_obstacle_parameters(self) -> None:
        positive_values = {
            "dynamic_obstacle_timeout_sec": self.dynamic_obstacle_timeout_sec,
            "dynamic_cbf_gain": self.dynamic_cbf_gain,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        non_negative_values = {
            "dynamic_obstacle_max_speed": self.dynamic_obstacle_max_speed,
            "dynamic_controlled_robot_radius": (
                self.dynamic_controlled_robot_radius
            ),
            "dynamic_robot_radius": self.dynamic_robot_radius,
            "dynamic_robot_safety_margin": self.dynamic_robot_safety_margin,
        }
        for name, value in non_negative_values.items():
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if not 0.0 <= self.dynamic_obstacle_velocity_filter_gain <= 1.0:
            raise ValueError(
                "dynamic_obstacle_velocity_filter_gain must be in [0, 1]"
            )

    def _parameter_array(self, name: str) -> List[float]:
        return self._coerce_float_array(self.get_parameter(name).value)

    def _parameter_int_array(self, name: str) -> List[int]:
        return self._coerce_int_array(self.get_parameter(name).value)

    def _coerce_float_array(self, value) -> List[float]:
        if isinstance(value, str):
            value = ast.literal_eval(value)
        return [float(item) for item in value]

    def _coerce_int_array(self, value) -> List[int]:
        if isinstance(value, str):
            value = ast.literal_eval(value)
        return [int(item) for item in value]

    def _sanitize_dynamic_robot_ids(self, robot_ids: List[int]) -> List[int]:
        sanitized = []
        for robot_id in robot_ids:
            if robot_id == self.robot_id or robot_id in sanitized:
                continue
            if robot_id <= 0:
                continue
            sanitized.append(robot_id)
        return sanitized

    def _obstacle_arrays_valid(
        self,
        obstacle_x: List[float],
        obstacle_y: List[float],
        obstacle_radius: List[float],
    ) -> bool:
        return obstacle_arrays_valid(obstacle_x, obstacle_y, obstacle_radius)

    def _obstacle_arrays_can_update(
        self,
        obstacle_x: List[float],
        obstacle_y: List[float],
        obstacle_radius: List[float],
    ) -> bool:
        if any(radius < 0.0 for radius in obstacle_radius):
            return False
        if not obstacle_x or not obstacle_y or not obstacle_radius:
            return True
        return self._obstacle_arrays_valid(
            obstacle_x,
            obstacle_y,
            obstacle_radius,
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

    def _target_pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        with self._target_pose_lock:
            self._latest_target_pose = (
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
            self._latest_target_pose_time = self.get_clock().now()

    def _reset_dynamic_robot_subscriptions(self, robot_ids: List[int]) -> None:
        for subscription in self._dynamic_robot_subscriptions.values():
            self.destroy_subscription(subscription)
        self._dynamic_robot_subscriptions = {}
        with self._dynamic_obstacle_lock:
            self._dynamic_robot_states = {}

        for robot_id in robot_ids:
            topic = f"/vrpn_mocap/dji_robot_{robot_id}/pose"
            self._dynamic_robot_subscriptions[robot_id] = self.create_subscription(
                PoseStamped,
                topic,
                self._make_dynamic_robot_pose_callback(robot_id),
                qos_profile_sensor_data,
                callback_group=self._callback_group,
            )
        if robot_ids:
            self.get_logger().info(
                f"Dynamic robot obstacle subscriptions: {robot_ids}"
            )

    def _make_dynamic_robot_pose_callback(self, robot_id: int):
        def callback(message: PoseStamped) -> None:
            self._dynamic_robot_pose_callback(robot_id, message)

        return callback

    def _dynamic_robot_pose_callback(
        self,
        robot_id: int,
        message: PoseStamped,
    ) -> None:
        pose = message.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        now = time.monotonic()
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return

        with self._dynamic_obstacle_lock:
            previous = self._dynamic_robot_states.get(robot_id)
            if previous is None:
                self._dynamic_robot_states[robot_id] = DynamicRobotState(
                    x=x,
                    y=y,
                    yaw=yaw,
                    timestamp=now,
                )
                return

            velocity_x = previous.velocity_x
            velocity_y = previous.velocity_y
            dt = now - previous.timestamp
            if 1e-3 <= dt <= self.dynamic_obstacle_timeout_sec:
                raw_velocity_x = (x - previous.x) / dt
                raw_velocity_y = (y - previous.y) / dt
                raw_speed = math.hypot(raw_velocity_x, raw_velocity_y)
                if math.isfinite(raw_speed):
                    if (
                        self.dynamic_obstacle_max_speed > 0.0
                        and raw_speed > self.dynamic_obstacle_max_speed
                    ):
                        scale = self.dynamic_obstacle_max_speed / raw_speed
                        raw_velocity_x *= scale
                        raw_velocity_y *= scale
                    gain = self.dynamic_obstacle_velocity_filter_gain
                    velocity_x = (
                        gain * raw_velocity_x
                        + (1.0 - gain) * previous.velocity_x
                    )
                    velocity_y = (
                        gain * raw_velocity_y
                        + (1.0 - gain) * previous.velocity_y
                    )

            self._dynamic_robot_states[robot_id] = DynamicRobotState(
                x=x,
                y=y,
                yaw=yaw,
                timestamp=now,
                previous_x=previous.x,
                previous_y=previous.y,
                previous_timestamp=previous.timestamp,
                velocity_x=velocity_x,
                velocity_y=velocity_y,
            )

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
        locked_goal_x = None
        locked_goal_y = None
        locked_desired_final_yaw = None
        next_tick_time = time.monotonic()

        try:
            while rclpy.ok():
                tick_start_time = time.monotonic()
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

                target_pose = (
                    self._get_fresh_target_pose()
                    if self.use_target_pose
                    else None
                )
                if target_pose is not None:
                    tx, ty, target_yaw = target_pose
                    cos_t = math.cos(target_yaw)
                    sin_t = math.sin(target_yaw)
                    goal_x = (
                        tx
                        + self.target_offset_x * cos_t
                        - self.target_offset_y * sin_t
                    )
                    goal_y = (
                        ty
                        + self.target_offset_x * sin_t
                        + self.target_offset_y * cos_t
                    )
                    desired_final_yaw = (
                        wrap_to_pi(target_yaw + self.target_orientation_offset)
                        if self.orient_to_target
                        else None
                    )
                else:
                    goal_x = request.target_x
                    goal_y = request.target_y
                    desired_final_yaw = None

                x, y, yaw = current_pose
                point_x, point_y = self._control_point(x, y, yaw)
                point_dx = goal_x - point_x
                point_dy = goal_y - point_y
                point_distance = math.hypot(point_dx, point_dy)
                final_distance = point_distance

                if state == SafeNavigationState.WAIT_FOR_POSE:
                    state = SafeNavigationState.TRACK_GOAL

                if state == SafeNavigationState.TRACK_GOAL:
                    if point_distance <= self.position_tolerance:
                        if desired_final_yaw is not None:
                            locked_goal_x = goal_x
                            locked_goal_y = goal_y
                            locked_desired_final_yaw = desired_final_yaw
                            self._stop_robot()
                            state = SafeNavigationState.ORIENT
                        else:
                            self._stop_robot()
                            state = SafeNavigationState.DONE
                    else:
                        command = self._compute_command(
                            point_x,
                            point_y,
                            goal_x,
                            goal_y,
                            yaw,
                            request.linear_speed,
                            request.angular_speed,
                            control_period,
                        )
                        if command is None:
                            self._stop_robot()
                        else:
                            linear_velocity, angular_velocity = command
                            self._publish_velocity(
                                linear_velocity,
                                angular_velocity,
                            )

                elif state == SafeNavigationState.ORIENT:
                    # ORIENT is a separate yaw controller after position QP
                    # navigation. Positional CBF constraints are not enforced
                    # here; use conservative radii if rotation can sweep body
                    # or stick geometry into obstacles.
                    if locked_desired_final_yaw is None:
                        self._stop_robot()
                        state = SafeNavigationState.DONE
                    elif (
                        abs(wrap_to_pi(locked_desired_final_yaw - yaw))
                        <= self.heading_tolerance
                    ):
                        self._stop_robot()
                        state = SafeNavigationState.DONE
                    else:
                        yaw_error = wrap_to_pi(locked_desired_final_yaw - yaw)
                        desired_angular_velocity = clamp(
                            self.orient_gain * yaw_error,
                            -request.angular_speed,
                            request.angular_speed,
                        )
                        linear_velocity = self._limit_rate(
                            0.0,
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
                        self._publish_velocity(linear_velocity, angular_velocity)

                elif state == SafeNavigationState.DONE:
                    self._stop_robot()

                self._publish_feedback(
                    goal_handle,
                    feedback,
                    state,
                    distance_remaining=point_distance,
                )

                if state == SafeNavigationState.DONE:
                    result_goal_x = (
                        goal_x if locked_goal_x is None else locked_goal_x
                    )
                    result_goal_y = (
                        goal_y if locked_goal_y is None else locked_goal_y
                    )
                    goal_handle.succeed()
                    result.success = True
                    result.message = (
                        f"Safely reached "
                        f"({result_goal_x:.2f}, {result_goal_y:.2f})."
                    )
                    result.final_distance = point_distance
                    self.get_logger().info(result.message)
                    return result

                self._record_loop_timing(tick_start_time)
                next_tick_time += control_period
                sleep_duration = max(0.0, next_tick_time - time.monotonic())
                if sleep_duration <= 0.0:
                    next_tick_time = time.monotonic()
                else:
                    time.sleep(sleep_duration)

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
        point_x: float,
        point_y: float,
        goal_x: float,
        goal_y: float,
        yaw: float,
        max_linear_speed: float,
        max_angular_speed: float,
        control_period: float,
    ) -> Optional[Tuple[float, float]]:
        u_nom_x, u_nom_y = self._compute_nominal_point_velocity(
            point_x,
            point_y,
            goal_x,
            goal_y,
        )
        dynamic_obstacles = self._get_dynamic_obstacles()
        if dynamic_obstacles is None:
            self._log_qp_failure("stale required dynamic obstacle data")
            return None
        qp_start_time = time.monotonic()
        qp_result = self._solve_clf_cbf_qp(
            point_x,
            point_y,
            goal_x,
            goal_y,
            u_nom_x,
            u_nom_y,
            dynamic_obstacles,
        )
        qp_solve_time_sec = time.monotonic() - qp_start_time
        if not qp_result.success:
            self._log_qp_failure(qp_result.status)
            return None
        if not all(
            math.isfinite(value)
            for value in (qp_result.u_x, qp_result.u_y, qp_result.delta)
        ):
            self._log_qp_failure("non-finite solution")
            return None

        desired_linear_velocity, desired_angular_velocity = (
            self._point_velocity_to_unicycle(
                qp_result.u_x,
                qp_result.u_y,
                yaw,
            )
        )
        self._last_qp_solve_time_sec = qp_solve_time_sec
        self._log_qp_diagnostics(qp_result, u_nom_x, u_nom_y)

        desired_linear_velocity = clamp(
            desired_linear_velocity,
            -max_linear_speed,
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

    def _compute_nominal_point_velocity(
        self,
        point_x: float,
        point_y: float,
        goal_x: float,
        goal_y: float,
    ) -> Tuple[float, float]:
        return compute_nominal_point_velocity(
            point_x,
            point_y,
            goal_x,
            goal_y,
            self.point_gain,
        )

    def _solve_clf_cbf_qp(
        self,
        point_x: float,
        point_y: float,
        goal_x: float,
        goal_y: float,
        u_nom_x: float,
        u_nom_y: float,
        dynamic_obstacles: List[DynamicCircularObstacle],
    ) -> QpResult:
        try:
            return solve_clf_cbf_qp(
                point_x,
                point_y,
                goal_x,
                goal_y,
                u_nom_x,
                u_nom_y,
                self._get_obstacles(),
                self.clf_gain,
                self.cbf_gain,
                self.slack_weight,
                self.max_point_speed,
                self.qp_solver,
                self.qp_verbose,
                dynamic_obstacles,
                self.dynamic_cbf_gain,
            )
        except Exception as exception:
            return QpResult(success=False, status=f"exception: {exception}")

    def _point_velocity_to_unicycle(
        self,
        point_velocity_x: float,
        point_velocity_y: float,
        yaw: float,
    ) -> Tuple[float, float]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * point_velocity_x + sin_yaw * point_velocity_y,
            (
                -sin_yaw * point_velocity_x
                + cos_yaw * point_velocity_y
            ) / self.lookahead_distance,
        )

    def _get_obstacles(self) -> List[CircularObstacle]:
        if not self.obstacles_enabled:
            return []
        if not self._obstacle_arrays_valid(
            self.obstacle_x,
            self.obstacle_y,
            self.obstacle_radius,
        ):
            return []

        obstacles = []
        for obstacle_x, obstacle_y, obstacle_radius in zip(
            self.obstacle_x,
            self.obstacle_y,
            self.obstacle_radius,
        ):
            obstacles.append(
                CircularObstacle(
                    obstacle_x,
                    obstacle_y,
                    obstacle_radius
                    + self.robot_safety_radius
                    + self.obstacle_safe_margin,
                )
            )
        return obstacles

    def _get_dynamic_obstacles(self) -> Optional[List[DynamicCircularObstacle]]:
        if not self.dynamic_robot_ids:
            return []

        now = time.monotonic()
        obstacles = []
        stale_ids = []
        with self._dynamic_obstacle_lock:
            states = {
                robot_id: self._dynamic_robot_states.get(robot_id)
                for robot_id in self.dynamic_robot_ids
            }

        for robot_id, state in states.items():
            if state is None:
                stale_ids.append(robot_id)
                continue
            age = now - state.timestamp
            if age > self.dynamic_obstacle_timeout_sec:
                stale_ids.append(robot_id)
                continue
            values = (
                state.x,
                state.y,
                state.velocity_x,
                state.velocity_y,
            )
            if not all(math.isfinite(value) for value in values):
                self._log_qp_failure(
                    f"invalid dynamic obstacle state for robot {robot_id}"
                )
                return None
            obstacles.append(
                DynamicCircularObstacle(
                    state.x,
                    state.y,
                    state.velocity_x,
                    state.velocity_y,
                    self.dynamic_controlled_robot_radius
                    + self.dynamic_robot_radius
                    + self.dynamic_robot_safety_margin,
                )
            )

        if stale_ids and self.dynamic_obstacles_required:
            self._log_qp_failure(
                f"stale dynamic obstacle data for robots {stale_ids}"
            )
            return None
        return obstacles

    def _log_qp_failure(self, status: str) -> None:
        now = time.monotonic()
        if now - self._last_qp_warning_time < 1.0:
            return
        self._last_qp_warning_time = now
        self.get_logger().warning(f"CLF-CBF-QP failed: {status}")

    def _log_qp_diagnostics(
        self,
        result: QpResult,
        u_nom_x: float,
        u_nom_y: float,
    ) -> None:
        if self.diagnostic_log_period_sec <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_diagnostic_log_time < self.diagnostic_log_period_sec:
            return
        self._last_diagnostic_log_time = now
        min_static_h = min((cbf.h for cbf in result.cbfs), default=math.inf)
        min_dynamic_h = min(
            (cbf.h for cbf in result.dynamic_cbfs),
            default=math.inf,
        )
        clf_value = result.clf.v if result.clf is not None else math.nan
        self.get_logger().info(
            "CLF-CBF-QP: "
            f"status={result.status}, "
            f"requested_rate={self.control_rate_hz:.1f}Hz, "
            f"measured_rate={self._measured_control_rate_hz:.1f}Hz, "
            f"qp_solve={1000.0 * self._last_qp_solve_time_sec:.1f}ms, "
            f"static_obstacles={len(result.cbfs)}, "
            f"dynamic_obstacles={len(result.dynamic_cbfs)}, "
            f"V={clf_value:.4f}, "
            f"delta={result.delta:.4f}, "
            f"min_static_h={min_static_h:.4f}, "
            f"min_dynamic_h={min_dynamic_h:.4f}, "
            f"u_nom=({u_nom_x:.3f}, {u_nom_y:.3f}), "
            f"u_safe=({result.u_x:.3f}, {result.u_y:.3f})"
        )

    def _record_loop_timing(self, tick_start_time: float) -> None:
        del tick_start_time
        self._loop_count += 1
        now = time.monotonic()
        elapsed = now - self._loop_rate_window_start
        if elapsed >= max(self.diagnostic_log_period_sec, 1.0):
            self._measured_control_rate_hz = self._loop_count / elapsed
            self._loop_count = 0
            self._loop_rate_window_start = now

    def _control_point(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> Tuple[float, float]:
        return (
            x + self.lookahead_distance * math.cos(yaw),
            y + self.lookahead_distance * math.sin(yaw),
        )

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

    def _get_fresh_target_pose(self) -> Optional[Tuple[float, float, float]]:
        with self._target_pose_lock:
            pose = self._latest_target_pose
            pose_time = self._latest_target_pose_time
        if pose is None or pose_time is None:
            return None
        pose_age = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        if pose_age > self.target_pose_timeout_sec:
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
