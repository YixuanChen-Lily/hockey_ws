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
from hockey_controller.clf_cbf_qp import QpResult
from hockey_controller.clf_cbf_qp import get_u_nom
from hockey_controller.clf_cbf_qp import obstacle_arrays_valid
from hockey_controller.clf_cbf_qp import solve_clf_cbf_qp
from hockey_controller.control_utils import clamp
from hockey_controller.control_utils import wrap_to_pi
from hockey_controller.control_utils import yaw_from_quaternion


class SafeNavigationState(Enum):
    WAIT_FOR_POSE = auto()
    TRACK_GOAL = auto()
    ORIENT = auto()
    DONE = auto()


@dataclass
class PoseObstacleState:
    x: float
    y: float
    yaw: float
    timestamp: float
    radius: float


@dataclass(frozen=True)
class PoseObstacleSpec:
    key: str
    topic: str
    radius: float


class SafeNavigationServer(Node):
    """Action server using approximate linearization for a unicycle robot."""

    CONTROL_RATE_HZ = 20.0
    POSITION_TOLERANCE = 0.01
    HEADING_TOLERANCE = 0.08
    POSE_TIMEOUT_SEC = 150.0
    TARGET_POSE_TIMEOUT_SEC = 150.0
    L = 0.25
    DEFAULT_K = 0.8
    CLF_GAMMA = 1.0
    CBF_GAMMA = 2.0
    W_DELTA = 100.0
    MAX_POINT_SPEED = 0.4
    QP_SOLVER = "cvxopt"
    DIAGNOSTIC_LOG_PERIOD_SEC = 1.0
    ORIENT_GAIN = 2.0
    TARGET_OFFSET_X = 0.0
    TARGET_OFFSET_Y = 0.0
    TARGET_ORIENTATION_OFFSET = 0.0
    POSE_OBSTACLE_TIMEOUT_SEC = 150.0
    POSE_OBSTACLE_CONTROLLED_ROBOT_RADIUS = 0.0
    POSE_OBSTACLE_ROBOT_RADIUS = 0.18
    POSE_OBSTACLE_ROBOT_SAFETY_MARGIN = 0.0
    DEFAULT_POSE_OBSTACLE_RADIUS = 0.10

    def __init__(self) -> None:
        super().__init__("safe_navigation_server")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("target_pose_topic", "")
        self.declare_parameter("action_name", "safe_navigate_to_point")
        self.declare_parameter("lookahead_distance", self.L)
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
        self.declare_parameter("orient_to_target", False)
        self.declare_parameter("use_target_pose", True)
        obstacle_robot_ids_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter(
            "obstacle_robot_ids",
            [],
            obstacle_robot_ids_descriptor,
        )
        obstacle_pose_topics_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter(
            "obstacle_pose_topics",
            [],
            obstacle_pose_topics_descriptor,
        )
        self.declare_parameter("obstacle_pose_radii", [], obstacle_array_descriptor)

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

        self.control_rate_hz = self.CONTROL_RATE_HZ
        self.position_tolerance = self.POSITION_TOLERANCE
        self.heading_tolerance = self.HEADING_TOLERANCE
        self.pose_timeout_sec = self.POSE_TIMEOUT_SEC
        self.target_pose_timeout_sec = self.TARGET_POSE_TIMEOUT_SEC
        self.l = float(self.get_parameter("lookahead_distance").value)
        self.K = self.DEFAULT_K
        self.clf_gamma = self.CLF_GAMMA
        self.cbf_gamma = self.CBF_GAMMA
        self.w_delta = self.W_DELTA
        self.max_point_speed = self.MAX_POINT_SPEED
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
        self.qp_solver = self.QP_SOLVER
        self.diagnostic_log_period_sec = self.DIAGNOSTIC_LOG_PERIOD_SEC
        self.orient_gain = self.ORIENT_GAIN
        self.target_offset_x = self.TARGET_OFFSET_X
        self.target_offset_y = self.TARGET_OFFSET_Y
        self.target_orientation_offset = self.TARGET_ORIENTATION_OFFSET
        self.orient_to_target = bool(self.get_parameter("orient_to_target").value)
        self.use_target_pose = bool(self.get_parameter("use_target_pose").value)
        self.obstacle_robot_ids = self._sanitize_obstacle_robot_ids(
            self._parameter_int_array("obstacle_robot_ids")
        )
        self.obstacle_pose_topics = self._sanitize_obstacle_pose_topics(
            self._parameter_string_array("obstacle_pose_topics")
        )
        self.obstacle_pose_radii = self._sanitize_obstacle_pose_radii(
            self._parameter_array("obstacle_pose_radii"),
            len(self.obstacle_pose_topics),
        )
        self.obstacle_pose_timeout_sec = self.POSE_OBSTACLE_TIMEOUT_SEC
        self.pose_obstacle_controlled_robot_radius = self.POSE_OBSTACLE_CONTROLLED_ROBOT_RADIUS
        self.pose_obstacle_robot_radius = self.POSE_OBSTACLE_ROBOT_RADIUS
        self.pose_obstacle_robot_safety_margin = self.POSE_OBSTACLE_ROBOT_SAFETY_MARGIN
        self.default_pose_obstacle_radius = self.DEFAULT_POSE_OBSTACLE_RADIUS
        self._validate_pose_obstacle_parameters()

        self._pose_lock = Lock()
        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._latest_pose_time = None
        self._target_pose_lock = Lock()
        self._latest_target_pose: Optional[Tuple[float, float, float]] = None
        self._latest_target_pose_time = None
        self._pose_obstacle_lock = Lock()
        self._pose_obstacle_states: Dict[str, PoseObstacleState] = {}
        self._pose_obstacle_specs: List[PoseObstacleSpec] = []
        self._pose_obstacle_subscriptions = {}
        self._goal_lock = Lock()
        self._goal_active = False
        self._last_qp_warning_time = 0.0
        self._last_diagnostic_log_time = 0.0
        self._last_qp_solve_time_sec = 0.0
        self._last_static_obstacle_count = 0
        self._last_pose_obstacle_count = 0
        self._last_min_obstacle_h = math.inf
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
        self._reset_pose_obstacle_subscriptions()
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
            f"  lookahead   = {self.l}\n"
            f"  offset      = ({self.target_offset_x}, {self.target_offset_y})\n"
            f"  orient      = {self.orient_to_target}\n"
            f"  use target pose = {self.use_target_pose}\n"
            f"  obstacles   = {len(self.obstacle_x)}\n"
            f"  obstacle robots = {self.obstacle_robot_ids}\n"
            f"  pose obstacle topics = {self.obstacle_pose_topics}"
        )

    def _handle_parameter_update(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "target_pose_topic":
                if parameter.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason="target_pose_topic must be a string",
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

        updated_obstacle_robot_ids = self.obstacle_robot_ids
        updated_obstacle_pose_topics = self.obstacle_pose_topics
        updated_obstacle_pose_radii = self.obstacle_pose_radii
        pose_topics_changed = False
        pose_radii_changed = False
        for parameter in parameters:
            try:
                if parameter.name == "obstacle_robot_ids":
                    updated_obstacle_robot_ids = self._sanitize_obstacle_robot_ids(
                        self._coerce_int_array(parameter.value)
                    )
                elif parameter.name == "obstacle_pose_topics":
                    pose_topics_changed = True
                    updated_obstacle_pose_topics = (
                        self._sanitize_obstacle_pose_topics(
                            self._coerce_string_array(parameter.value)
                        )
                    )
                elif parameter.name == "obstacle_pose_radii":
                    pose_radii_changed = True
                    updated_obstacle_pose_radii = (
                        self._sanitize_obstacle_pose_radii(
                            self._coerce_float_array(parameter.value),
                            len(updated_obstacle_pose_topics),
                        )
                    )
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=(
                        "obstacle_robot_ids must be integers, "
                        "obstacle_pose_topics must be strings, and "
                        "obstacle_pose_radii must be non-negative numbers"
                    ),
                )

        if pose_topics_changed and not pose_radii_changed:
            updated_obstacle_pose_radii = self._sanitize_obstacle_pose_radii(
                [],
                len(updated_obstacle_pose_topics),
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
            if parameter.name == "obstacle_safe_margin":
                self.obstacle_safe_margin = float(parameter.value)
            elif parameter.name == "lookahead_distance":
                self.l = float(parameter.value)
            elif parameter.name == "robot_safety_radius":
                self.robot_safety_radius = float(parameter.value)
            elif parameter.name == "obstacle_x":
                self.obstacle_x = updated_obstacle_x
            elif parameter.name == "obstacle_y":
                self.obstacle_y = updated_obstacle_y
            elif parameter.name == "obstacle_radius":
                self.obstacle_radius = updated_obstacle_radius
            elif parameter.name == "target_pose_topic":
                new_topic = str(parameter.value)
                if not new_topic:
                    new_topic = "/vrpn_mocap/hockey_sticks_1/pose"
                self._reset_target_pose_subscription(new_topic)
            elif parameter.name == "orient_to_target":
                self.orient_to_target = bool(parameter.value)
            elif parameter.name == "use_target_pose":
                self.use_target_pose = bool(parameter.value)
            elif parameter.name == "obstacle_robot_ids":
                self.obstacle_robot_ids = updated_obstacle_robot_ids
                self._reset_pose_obstacle_subscriptions()
            elif parameter.name == "obstacle_pose_topics":
                self.obstacle_pose_topics = updated_obstacle_pose_topics
                self.obstacle_pose_radii = updated_obstacle_pose_radii
                self._reset_pose_obstacle_subscriptions()
            elif parameter.name == "obstacle_pose_radii":
                self.obstacle_pose_radii = updated_obstacle_pose_radii
                self._reset_pose_obstacle_subscriptions()

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
        positive_values = (
            self.l,
            self.K,
            self.clf_gamma,
            self.cbf_gamma,
            self.w_delta,
            self.max_point_speed,
            self.heading_tolerance,
            self.target_pose_timeout_sec,
        )
        if any(value <= 0.0 for value in positive_values):
            raise ValueError("safe navigation controller constants must be positive")
        if self.robot_safety_radius < 0.0:
            raise ValueError("robot_safety_radius must be non-negative")
        if self.obstacle_safe_margin < 0.0:
            raise ValueError("obstacle_safe_margin must be non-negative")
        if not obstacle_arrays_valid(
            self.obstacle_x,
            self.obstacle_y,
            self.obstacle_radius,
        ):
            raise ValueError(
                "obstacle_x, obstacle_y, and obstacle_radius must have "
                "equal length and non-negative radii"
            )

    def _validate_pose_obstacle_parameters(self) -> None:
        if self.obstacle_pose_timeout_sec <= 0.0:
            raise ValueError("pose obstacle controller constants must be positive")
        non_negative_values = (
            self.pose_obstacle_controlled_robot_radius,
            self.pose_obstacle_robot_radius,
            self.pose_obstacle_robot_safety_margin,
        )
        if any(value < 0.0 for value in non_negative_values):
            raise ValueError(
                "pose obstacle radius constants must be non-negative"
            )

    def _parameter_array(self, name: str) -> List[float]:
        return self._coerce_float_array(self.get_parameter(name).value)

    def _parameter_int_array(self, name: str) -> List[int]:
        return self._coerce_int_array(self.get_parameter(name).value)

    def _parameter_string_array(self, name: str) -> List[str]:
        return self._coerce_string_array(self.get_parameter(name).value)

    def _coerce_float_array(self, value) -> List[float]:
        if isinstance(value, str):
            value = ast.literal_eval(value)
        return [float(item) for item in value]

    def _coerce_int_array(self, value) -> List[int]:
        if isinstance(value, str):
            value = ast.literal_eval(value)
        return [int(item) for item in value]

    def _coerce_string_array(self, value) -> List[str]:
        if isinstance(value, str):
            value = ast.literal_eval(value)
        return [str(item) for item in value]

    def _sanitize_obstacle_robot_ids(self, robot_ids: List[int]) -> List[int]:
        sanitized = []
        for robot_id in robot_ids:
            if robot_id == self.robot_id or robot_id in sanitized:
                continue
            if robot_id <= 0:
                continue
            sanitized.append(robot_id)
        return sanitized

    def _sanitize_obstacle_pose_topics(self, topics: List[str]) -> List[str]:
        sanitized = []
        for topic in topics:
            topic = topic.strip()
            if not topic or topic in sanitized:
                continue
            sanitized.append(topic)
        return sanitized

    def _sanitize_obstacle_pose_radii(
        self,
        radii: List[float],
        topic_count: int,
    ) -> List[float]:
        if any(radius < 0.0 for radius in radii):
            raise ValueError("pose obstacle radii must be non-negative")
        if not radii:
            return [self.DEFAULT_POSE_OBSTACLE_RADIUS] * topic_count
        if len(radii) != topic_count:
            raise ValueError("pose obstacle radii must match topic count")
        return radii

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
        return obstacle_arrays_valid(
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

    def _reset_pose_obstacle_subscriptions(self) -> None:
        for subscription in self._pose_obstacle_subscriptions.values():
            self.destroy_subscription(subscription)
        self._pose_obstacle_subscriptions = {}
        with self._pose_obstacle_lock:
            self._pose_obstacle_states = {}

        specs: List[PoseObstacleSpec] = []
        robot_radius = (
            self.pose_obstacle_controlled_robot_radius
            + self.pose_obstacle_robot_radius
            + self.pose_obstacle_robot_safety_margin
        )
        for robot_id in self.obstacle_robot_ids:
            specs.append(
                PoseObstacleSpec(
                    key=f"robot_{robot_id}",
                    topic=f"/vrpn_mocap/dji_robot_{robot_id}/pose",
                    radius=robot_radius,
                )
            )

        for index, topic in enumerate(self.obstacle_pose_topics):
            object_radius = self.obstacle_pose_radii[index]
            specs.append(
                PoseObstacleSpec(
                    key=f"object_{index}",
                    topic=topic,
                    radius=(
                        self.pose_obstacle_controlled_robot_radius
                        + object_radius
                        + self.pose_obstacle_robot_safety_margin
                    ),
                )
            )

        self._pose_obstacle_specs = specs
        for spec in specs:
            self._pose_obstacle_subscriptions[spec.key] = self.create_subscription(
                PoseStamped,
                spec.topic,
                lambda message, spec=spec: (
                    self._pose_obstacle_callback(spec, message)
                ),
                qos_profile_sensor_data,
                callback_group=self._callback_group,
            )
        if specs:
            self.get_logger().info(
                "Pose obstacle subscriptions: "
                f"{[(spec.key, spec.topic, round(spec.radius, 3)) for spec in specs]}"
            )

    def _pose_obstacle_callback(
        self,
        spec: PoseObstacleSpec,
        message: PoseStamped,
    ) -> None:
        pose = message.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)
        now = time.monotonic()
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return

        with self._pose_obstacle_lock:
            self._pose_obstacle_states[spec.key] = PoseObstacleState(
                x=x,
                y=y,
                yaw=yaw,
                timestamp=now,
                radius=spec.radius,
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
                        self._publish_velocity(0.0, desired_angular_velocity)

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

                self._record_loop_timing()
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
        p_x: float,
        p_y: float,
        g_x: float,
        g_y: float,
        yaw: float,
        max_linear_speed: float,
        max_angular_speed: float,
    ) -> Optional[Tuple[float, float]]:
        u_nom_x, u_nom_y = self._get_u_nom(
            p_x,
            p_y,
            g_x,
            g_y,
        )
        qp_start_time = time.monotonic()
        qp_result = self._solve_clf_cbf_qp(
            p_x,
            p_y,
            g_x,
            g_y,
            u_nom_x,
            u_nom_y,
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
            self._pdot_to_v_and_w(
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

        return desired_linear_velocity, desired_angular_velocity

    def _get_u_nom(
        self,
        p_x: float,
        p_y: float,
        g_x: float,
        g_y: float,
    ) -> Tuple[float, float]:
        return get_u_nom(
            p_x,
            p_y,
            g_x,
            g_y,
            self.K,
        )

    def _solve_clf_cbf_qp(
        self,
        p_x: float,
        p_y: float,
        g_x: float,
        g_y: float,
        u_nom_x: float,
        u_nom_y: float,
    ) -> QpResult:
        try:
            obstacles = self._get_obstacles()
            self._last_min_obstacle_h = min(
                (
                    (p_x - obstacle.x) ** 2
                    + (p_y - obstacle.y) ** 2
                    - obstacle.radius**2
                    for obstacle in obstacles
                ),
                default=math.inf,
            )
            return solve_clf_cbf_qp(
                p_x,
                p_y,
                g_x,
                g_y,
                u_nom_x,
                u_nom_y,
                obstacles,
                self.clf_gamma,
                self.cbf_gamma,
                self.w_delta,
                self.max_point_speed,
                self.qp_solver,
            )
        except Exception as exception:
            return QpResult(success=False, status=f"exception: {exception}")

    def _pdot_to_v_and_w(
        self,
        pdot_x: float,
        pdot_y: float,
        theta: float,
    ) -> Tuple[float, float]:
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        v = cos_theta * pdot_x + sin_theta * pdot_y
        w = (-sin_theta * pdot_x + cos_theta * pdot_y) / self.l
        return v, w

    def _get_obstacles(self) -> List[CircularObstacle]:
        obstacles = []
        pose_obstacle_count = 0
        if not obstacle_arrays_valid(
            self.obstacle_x,
            self.obstacle_y,
            self.obstacle_radius,
        ):
            self._last_pose_obstacle_count = pose_obstacle_count
            return obstacles

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

        now = time.monotonic()
        stale_keys = []
        with self._pose_obstacle_lock:
            states = {
                spec.key: self._pose_obstacle_states.get(spec.key)
                for spec in self._pose_obstacle_specs
            }

        for key, state in states.items():
            if state is None:
                stale_keys.append(key)
                continue
            age = now - state.timestamp
            if age > self.obstacle_pose_timeout_sec:
                stale_keys.append(key)
                continue
            values = (state.x, state.y, state.radius)
            if not all(math.isfinite(value) for value in values):
                self._log_qp_failure(
                    f"invalid pose obstacle state for {key}"
                )
                continue
            obstacles.append(
                CircularObstacle(
                    state.x,
                    state.y,
                    state.radius,
                )
            )
            pose_obstacle_count += 1

        if stale_keys:
            self._log_qp_failure(
                f"stale pose obstacle data for {stale_keys}"
            )
        self._last_static_obstacle_count = len(obstacles) - pose_obstacle_count
        self._last_pose_obstacle_count = pose_obstacle_count
        return obstacles

    def _log_qp_failure(self, status: str) -> None:
        now = time.monotonic()
        if now - self._last_qp_warning_time < 1.0:
            return
        self._last_qp_warning_time = now
        self.get_logger().warning(
            "CLF-CBF-QP failed: "
            f"{status}, "
            f"static_obstacles={self._last_static_obstacle_count}, "
            f"pose_obstacles={self._last_pose_obstacle_count}, "
            f"min_h={self._last_min_obstacle_h:.4f}"
        )

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
        clf_value = result.clf.v if result.clf is not None else math.nan
        self.get_logger().info(
            "CLF-CBF-QP: "
            f"status={result.status}, "
            f"requested_rate={self.control_rate_hz:.1f}Hz, "
            f"measured_rate={self._measured_control_rate_hz:.1f}Hz, "
            f"qp_solve={1000.0 * self._last_qp_solve_time_sec:.1f}ms, "
            f"static_obstacles={len(result.cbfs)}, "
            f"pose_obstacles={self._last_pose_obstacle_count}, "
            f"V={clf_value:.4f}, "
            f"delta={result.delta:.4f}, "
            f"min_static_h={min_static_h:.4f}, "
            f"u_nom=({u_nom_x:.3f}, {u_nom_y:.3f}), "
            f"u_safe=({result.u_x:.3f}, {result.u_y:.3f})"
        )

    def _record_loop_timing(self) -> None:
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
            x + self.l * math.cos(yaw),
            y + self.l * math.sin(yaw),
        )

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
