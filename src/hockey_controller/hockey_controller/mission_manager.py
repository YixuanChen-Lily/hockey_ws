#!/usr/bin/env python3

import math
from threading import Event, Lock, Thread
from time import monotonic
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    Marker = None
    MarkerArray = None

from hockey_interfaces.action import NavigateToPoint
from hockey_interfaces.action import Spin
try:
    from hockey_interfaces.action import GripperControl
    from hockey_interfaces.action import MoveArm
except ImportError:
    GripperControl = None
    MoveArm = None
from hockey_controller.cushion_parking_planner import CushionGeometry
from hockey_controller.cushion_parking_planner import ParkingPlannerConfig
from hockey_controller.cushion_parking_planner import cushion_axes
from hockey_controller.cushion_parking_planner import cushion_radius_layers
from hockey_controller.cushion_parking_planner import plan_parking_route
from hockey_controller.navigation_server import clamp
from hockey_controller.navigation_server import wrap_to_pi
from hockey_controller.navigation_server import yaw_from_quaternion


class MissionManager(Node):
    """Coordinate parking, stick pickup, navigation, and spinning tasks."""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        self.declare_parameter("navigation_action", "navigate_to_point")
        self.declare_parameter("safe_navigation_action", "safe_navigate_to_point")
        self.declare_parameter("spin_action", "spin")
        self.declare_parameter("arm_action", "control_arm")
        self.declare_parameter("gripper_action", "control_gripper")
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("cushion_pose_topic", "")
        self.declare_parameter("parking_enabled", True)
        self.declare_parameter("target_x", 1.0)
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("safe_target_x", 1.0)
        self.declare_parameter("safe_target_y", 0.0)
        self.declare_parameter("cushion_length", 1.0)
        self.declare_parameter("cushion_width", 0.12)
        self.declare_parameter("parking_front_axis", "y")
        self.declare_parameter("front_normal_sign", -1.0)
        self.declare_parameter("front_side_threshold", 0.0)
        self.declare_parameter("side_clearance", 0.35)
        self.declare_parameter("front_clearance", 0.35)
        self.declare_parameter("desired_normal_distance", 0.35)
        self.declare_parameter("tangential_offset", 0.0)
        self.declare_parameter("parking_lateral_offset", 0.0)
        self.declare_parameter("pre_park_backoff", 0.40)
        self.declare_parameter("parking_robot_safety_radius", 0.20)
        self.declare_parameter("stick_safety_extension", 0.0)
        self.declare_parameter("parking_safety_margin", 0.10)
        self.declare_parameter("cushion_circle_spacing", 0.20)
        self.declare_parameter("cushion_obstacle_axis", "local_x")
        self.declare_parameter("cushion_obstacle_radius_override", -1.0)
        self.declare_parameter("parking_lookahead_distance", 0.25)
        self.declare_parameter("final_approach_speed", 0.12)
        self.declare_parameter("final_approach_point_gain", 0.35)
        self.declare_parameter("align_gain", 2.0)
        self.declare_parameter("align_timeout_sec", 8.0)
        self.declare_parameter("final_yaw_tolerance", 0.08)
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("visualization_frame", "map")
        self.declare_parameter("rotations", 1)
        self.declare_parameter("linear_speed", 0.4)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("navigation_timeout_sec", 30.0)
        self.declare_parameter("safe_navigation_timeout_sec", 30.0)
        self.declare_parameter("spin_timeout_sec", 15.0)
        self.declare_parameter("action_wait_timeout_sec", 5.0)

        # ================================================================
        # Hockey-stick pickup and ready-position parameters.
        #
        # Keep this task disabled until all arm X/Z positions have been
        # measured on the real robot. MoveArm uses the coordinate convention
        # and units provided by the RoboMaster arm driver.
        # ================================================================
        self.declare_parameter("stick_setup_enabled", False)

        # Step 1: move the arm end effector to the stick pickup position.
        self.declare_parameter("grab_arm_x", 0.0)
        self.declare_parameter("grab_arm_z", 0.0)
        self.declare_parameter("grab_arm_relative", False)
        self.declare_parameter("grab_arm_timeout_sec", 8.0)
        self.declare_parameter("grab_arm_settle_sec", 0.3)

        # Step 2: close the gripper and allow the stick to settle in it.
        self.declare_parameter("gripper_close_power", 0.5)
        self.declare_parameter("gripper_close_timeout_sec", 5.0)
        self.declare_parameter("gripper_close_settle_sec", 0.5)

        # Step 3: lift the captured stick clear of the floor/fixture.
        self.declare_parameter("lift_arm_x", 1.0)
        self.declare_parameter("lift_arm_z", 2.0)
        self.declare_parameter("lift_arm_relative", False)
        self.declare_parameter("lift_arm_timeout_sec", 8.0)
        self.declare_parameter("lift_arm_settle_sec", 0.3)

        # Step 4: open-loop reverse motion. The commanded linear velocity is
        # -backward_distance / backward_duration_sec and is sent as Twist.
        self.declare_parameter("backward_distance", 0.30)
        self.declare_parameter("backward_duration_sec", 2.0)
        self.declare_parameter("backward_publish_rate_hz", 20.0)
        self.declare_parameter("backward_max_speed", 0.30)

        # Step 5: lower the arm into the ready-to-hit end-effector position.
        self.declare_parameter("ready_arm_x", 0.0)
        self.declare_parameter("ready_arm_z", 0.0)
        self.declare_parameter("ready_arm_relative", False)
        self.declare_parameter("ready_arm_timeout_sec", 8.0)
        self.declare_parameter("ready_arm_settle_sec", 0.3)

        self.navigation_action = str(
            self.get_parameter("navigation_action").value
        )
        self.safe_navigation_action = str(
            self.get_parameter("safe_navigation_action").value
        )
        self.spin_action = str(self.get_parameter("spin_action").value)
        self.arm_action = str(self.get_parameter("arm_action").value)
        self.gripper_action = str(self.get_parameter("gripper_action").value)
        self.robot_id = int(self.get_parameter("robot_id").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        cushion_pose_topic = str(self.get_parameter("cushion_pose_topic").value)
        self.pose_topic = (
            pose_topic
            if pose_topic
            else f"/vrpn_mocap/dji_robot_{self.robot_id}/pose"
        )
        self.cushion_pose_topic = (
            cushion_pose_topic
            if cushion_pose_topic
            else "/vrpn_mocap/hockey_sticks_1/pose"
        )
        self.cmd_vel_topic = f"/robot{self.robot_id}/cmd_vel"
        self.parking_enabled = bool(self.get_parameter("parking_enabled").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.safe_target_x = float(self.get_parameter("safe_target_x").value)
        self.safe_target_y = float(self.get_parameter("safe_target_y").value)
        self._reload_parking_parameters()
        self.rotations = int(self.get_parameter("rotations").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.navigation_timeout_sec = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        self.safe_navigation_timeout_sec = float(
            self.get_parameter("safe_navigation_timeout_sec").value
        )
        self.spin_timeout_sec = float(self.get_parameter("spin_timeout_sec").value)
        self.action_wait_timeout_sec = float(
            self.get_parameter("action_wait_timeout_sec").value
        )
        self._reload_stick_setup_parameters()

        self._lock = Lock()
        self._running = False
        self._stop_requested = False
        self._worker: Optional[Thread] = None
        self._active_goal_handle = None
        self._pose_lock = Lock()
        self._latest_pose: Optional[Tuple[float, float, float]] = None
        self._latest_pose_time = None
        self._cushion_pose_lock = Lock()
        self._latest_cushion_pose: Optional[Tuple[float, float, float]] = None
        self._latest_cushion_pose_time = None
        self._callback_group = ReentrantCallbackGroup()

        self._status_publisher = self.create_publisher(
            String,
            "mission/status",
            10,
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self._parking_marker_publisher = None
        if MarkerArray is not None:
            self._parking_marker_publisher = self.create_publisher(
                MarkerArray,
                "mission/parking_markers",
                10,
            )
        self._pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._cushion_pose_subscription = self.create_subscription(
            PoseStamped,
            self.cushion_pose_topic,
            self._cushion_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._start_service = self.create_service(
            Trigger,
            "mission/start",
            self._handle_start,
            callback_group=self._callback_group,
        )
        self._stop_service = self.create_service(
            Trigger,
            "mission/stop",
            self._handle_stop,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._handle_parameter_update)
        self._navigation_client = ActionClient(
            self,
            NavigateToPoint,
            self.navigation_action,
            callback_group=self._callback_group,
        )
        self._safe_navigation_client = ActionClient(
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
        self._arm_client = None
        if MoveArm is not None:
            self._arm_client = ActionClient(
                self,
                MoveArm,
                self.arm_action,
                callback_group=self._callback_group,
            )
        self._gripper_client = None
        if GripperControl is not None:
            self._gripper_client = ActionClient(
                self,
                GripperControl,
                self.gripper_action,
                callback_group=self._callback_group,
            )
        self._safe_nav_parameter_client = self.create_client(
            SetParameters,
            "safe_navigation_server/set_parameters",
            callback_group=self._callback_group,
        )

        self._publish_status("IDLE")
        self.get_logger().info(
            "Mission manager ready. Call mission/start in this node namespace.\n"
            f"  step1 action = {self.safe_navigation_action}\n"
            f"  step2 action = {self.spin_action}\n"
            f"  safe target  = "
            f"({self.safe_target_x:.2f}, {self.safe_target_y:.2f})\n"
            f"  parking      = {self.parking_enabled}\n"
            f"  stick setup  = {self.stick_setup_enabled}\n"
            f"  rotations    = {self.rotations}"
        )

    def _handle_parameter_update(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "cushion_pose_topic":
                new_topic = str(parameter.value)
                if not new_topic:
                    new_topic = "/vrpn_mocap/hockey_sticks_1/pose"
                self._reset_cushion_pose_subscription(new_topic)

        return SetParametersResult(successful=True)

    def _reset_cushion_pose_subscription(self, topic: str) -> None:
        if topic == self.cushion_pose_topic:
            return
        self.destroy_subscription(self._cushion_pose_subscription)
        with self._cushion_pose_lock:
            self._latest_cushion_pose = None
            self._latest_cushion_pose_time = None
        self.cushion_pose_topic = topic
        self._cushion_pose_subscription = self.create_subscription(
            PoseStamped,
            self.cushion_pose_topic,
            self._cushion_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"Updated cushion pose subscription: {self.cushion_pose_topic}"
        )

    def _handle_start(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        with self._lock:
            if self._running:
                response.success = False
                response.message = "Mission is already running"
                return response

            self._reload_parameters()
            parameters_valid, parameter_message = (
                self._validate_stick_setup_parameters()
            )
            if not parameters_valid:
                response.success = False
                response.message = parameter_message
                return response
            self._stop_requested = False
            self._running = True

        self._worker = Thread(target=self._run_mission, daemon=True)
        self._worker.start()

        response.success = True
        response.message = "Mission started"
        return response

    def _handle_stop(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        with self._lock:
            if not self._running:
                response.success = False
                response.message = "Mission is not running"
                return response

            self._stop_requested = True
            active_goal_handle = self._active_goal_handle

        if active_goal_handle is not None:
            active_goal_handle.cancel_goal_async()
        self._stop_robot()

        self._publish_status("MISSION_STOPPING")
        response.success = True
        response.message = "Mission stop requested"
        return response

    def _reload_parameters(self) -> None:
        self.navigation_action = str(
            self.get_parameter("navigation_action").value
        )
        self.safe_navigation_action = str(
            self.get_parameter("safe_navigation_action").value
        )
        self.spin_action = str(self.get_parameter("spin_action").value)
        self.parking_enabled = bool(self.get_parameter("parking_enabled").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.safe_target_x = float(self.get_parameter("safe_target_x").value)
        self.safe_target_y = float(self.get_parameter("safe_target_y").value)
        self._reload_parking_parameters()
        self.rotations = int(self.get_parameter("rotations").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.navigation_timeout_sec = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        self.safe_navigation_timeout_sec = float(
            self.get_parameter("safe_navigation_timeout_sec").value
        )
        self.spin_timeout_sec = float(self.get_parameter("spin_timeout_sec").value)
        self.action_wait_timeout_sec = float(
            self.get_parameter("action_wait_timeout_sec").value
        )
        self._reload_stick_setup_parameters()

    def _reload_stick_setup_parameters(self) -> None:
        self.stick_setup_enabled = bool(
            self.get_parameter("stick_setup_enabled").value
        )
        self.grab_arm_x = float(self.get_parameter("grab_arm_x").value)
        self.grab_arm_z = float(self.get_parameter("grab_arm_z").value)
        self.grab_arm_relative = bool(
            self.get_parameter("grab_arm_relative").value
        )
        self.grab_arm_timeout_sec = float(
            self.get_parameter("grab_arm_timeout_sec").value
        )
        self.grab_arm_settle_sec = float(
            self.get_parameter("grab_arm_settle_sec").value
        )
        self.gripper_close_power = float(
            self.get_parameter("gripper_close_power").value
        )
        self.gripper_close_timeout_sec = float(
            self.get_parameter("gripper_close_timeout_sec").value
        )
        self.gripper_close_settle_sec = float(
            self.get_parameter("gripper_close_settle_sec").value
        )
        self.lift_arm_x = float(self.get_parameter("lift_arm_x").value)
        self.lift_arm_z = float(self.get_parameter("lift_arm_z").value)
        self.lift_arm_relative = bool(
            self.get_parameter("lift_arm_relative").value
        )
        self.lift_arm_timeout_sec = float(
            self.get_parameter("lift_arm_timeout_sec").value
        )
        self.lift_arm_settle_sec = float(
            self.get_parameter("lift_arm_settle_sec").value
        )
        self.backward_distance = float(
            self.get_parameter("backward_distance").value
        )
        self.backward_duration_sec = float(
            self.get_parameter("backward_duration_sec").value
        )
        self.backward_publish_rate_hz = float(
            self.get_parameter("backward_publish_rate_hz").value
        )
        self.backward_max_speed = float(
            self.get_parameter("backward_max_speed").value
        )
        self.ready_arm_x = float(self.get_parameter("ready_arm_x").value)
        self.ready_arm_z = float(self.get_parameter("ready_arm_z").value)
        self.ready_arm_relative = bool(
            self.get_parameter("ready_arm_relative").value
        )
        self.ready_arm_timeout_sec = float(
            self.get_parameter("ready_arm_timeout_sec").value
        )
        self.ready_arm_settle_sec = float(
            self.get_parameter("ready_arm_settle_sec").value
        )

    def _validate_stick_setup_parameters(self) -> Tuple[bool, str]:
        if not self.stick_setup_enabled:
            return True, "Stick setup is disabled"
        if MoveArm is None or GripperControl is None:
            return (
                False,
                "Stick setup requires MoveArm and GripperControl actions. "
                "Rebuild hockey_interfaces first.",
            )

        finite_values = {
            "grab_arm_x": self.grab_arm_x,
            "grab_arm_z": self.grab_arm_z,
            "grab_arm_timeout_sec": self.grab_arm_timeout_sec,
            "grab_arm_settle_sec": self.grab_arm_settle_sec,
            "gripper_close_power": self.gripper_close_power,
            "gripper_close_timeout_sec": self.gripper_close_timeout_sec,
            "gripper_close_settle_sec": self.gripper_close_settle_sec,
            "lift_arm_x": self.lift_arm_x,
            "lift_arm_z": self.lift_arm_z,
            "lift_arm_timeout_sec": self.lift_arm_timeout_sec,
            "lift_arm_settle_sec": self.lift_arm_settle_sec,
            "backward_distance": self.backward_distance,
            "backward_duration_sec": self.backward_duration_sec,
            "backward_publish_rate_hz": self.backward_publish_rate_hz,
            "backward_max_speed": self.backward_max_speed,
            "ready_arm_x": self.ready_arm_x,
            "ready_arm_z": self.ready_arm_z,
            "ready_arm_timeout_sec": self.ready_arm_timeout_sec,
            "ready_arm_settle_sec": self.ready_arm_settle_sec,
        }
        for name, value in finite_values.items():
            if not math.isfinite(value):
                return (
                    False,
                    f"Invalid stick setup parameter: {name} is not finite",
                )

        positive_values = {
            "grab_arm_timeout_sec": self.grab_arm_timeout_sec,
            "gripper_close_timeout_sec": self.gripper_close_timeout_sec,
            "lift_arm_timeout_sec": self.lift_arm_timeout_sec,
            "backward_distance": self.backward_distance,
            "backward_duration_sec": self.backward_duration_sec,
            "backward_publish_rate_hz": self.backward_publish_rate_hz,
            "backward_max_speed": self.backward_max_speed,
            "ready_arm_timeout_sec": self.ready_arm_timeout_sec,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                return (
                    False,
                    f"Invalid stick setup parameter: {name} must be > 0",
                )

        settle_values = {
            "grab_arm_settle_sec": self.grab_arm_settle_sec,
            "gripper_close_settle_sec": self.gripper_close_settle_sec,
            "lift_arm_settle_sec": self.lift_arm_settle_sec,
            "ready_arm_settle_sec": self.ready_arm_settle_sec,
        }
        for name, value in settle_values.items():
            if value < 0.0:
                return (
                    False,
                    f"Invalid stick setup parameter: {name} must be >= 0",
                )

        if not 0.0 <= self.gripper_close_power <= 1.0:
            return False, "gripper_close_power must be in [0, 1]"

        requested_speed = self.backward_distance / self.backward_duration_sec
        if requested_speed > self.backward_max_speed:
            return (
                False,
                "backward_distance / backward_duration_sec exceeds "
                "backward_max_speed",
            )

        return True, "Stick setup parameters are valid"

    def _reload_parking_parameters(self) -> None:
        self.cushion_length = float(self.get_parameter("cushion_length").value)
        self.cushion_width = float(self.get_parameter("cushion_width").value)
        self.parking_front_axis = str(
            self.get_parameter("parking_front_axis").value
        )
        self.front_normal_sign = float(
            self.get_parameter("front_normal_sign").value
        )
        self.front_side_threshold = float(
            self.get_parameter("front_side_threshold").value
        )
        self.side_clearance = float(self.get_parameter("side_clearance").value)
        self.front_clearance = float(self.get_parameter("front_clearance").value)
        self.desired_normal_distance = float(
            self.get_parameter("desired_normal_distance").value
        )
        self.tangential_offset = float(
            self.get_parameter("tangential_offset").value
        )
        self.parking_lateral_offset = float(
            self.get_parameter("parking_lateral_offset").value
        )
        self.pre_park_backoff = float(
            self.get_parameter("pre_park_backoff").value
        )
        self.parking_robot_safety_radius = float(
            self.get_parameter("parking_robot_safety_radius").value
        )
        self.stick_safety_extension = float(
            self.get_parameter("stick_safety_extension").value
        )
        self.parking_safety_margin = float(
            self.get_parameter("parking_safety_margin").value
        )
        self.cushion_circle_spacing = float(
            self.get_parameter("cushion_circle_spacing").value
        )
        self.cushion_obstacle_axis = str(
            self.get_parameter("cushion_obstacle_axis").value
        )
        self.cushion_obstacle_radius_override = float(
            self.get_parameter("cushion_obstacle_radius_override").value
        )
        self.parking_lookahead_distance = float(
            self.get_parameter("parking_lookahead_distance").value
        )
        self.final_approach_speed = float(
            self.get_parameter("final_approach_speed").value
        )
        self.final_approach_point_gain = float(
            self.get_parameter("final_approach_point_gain").value
        )
        self.align_gain = float(self.get_parameter("align_gain").value)
        self.align_timeout_sec = float(
            self.get_parameter("align_timeout_sec").value
        )
        self.final_yaw_tolerance = float(
            self.get_parameter("final_yaw_tolerance").value
        )
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.visualization_frame = str(
            self.get_parameter("visualization_frame").value
        )

    def _run_mission(self) -> None:
        try:
            if self.parking_enabled:
                self._run_parking_mission()
                return

            self._publish_status("STEP1_SAFE_NAVIGATE")
            if self._is_stop_requested():
                self._publish_status("MISSION_STOPPED")
                return

            safe_navigation_success, safe_navigation_message = (
                self._run_safe_navigation_step()
            )
            if self._is_stop_requested():
                self._publish_status("MISSION_STOPPED")
                return
            if not safe_navigation_success:
                self._publish_status("MISSION_FAILED")
                self.get_logger().error(
                    f"Step1 safe navigation failed: {safe_navigation_message}"
                )
                return

            self.get_logger().info(
                "Step1 safe navigation succeeded. Transitioning to step2 spin."
            )

            self._publish_status("STEP2_SPIN")
            if self._is_stop_requested():
                self._publish_status("MISSION_STOPPED")
                return

            spin_success, spin_message = self._run_spin_step()
            if self._is_stop_requested():
                self._publish_status("MISSION_STOPPED")
                return
            if not spin_success:
                self._publish_status("MISSION_FAILED")
                self.get_logger().error(f"Step2 spin failed: {spin_message}")
                return

            self._publish_status("MISSION_DONE")
            self.get_logger().info("Mission completed successfully.")

        except Exception as exception:
            self._publish_status("MISSION_FAILED")
            self.get_logger().error(f"Mission exception: {exception}")

        finally:
            with self._lock:
                self._running = False
                self._stop_requested = False
                self._active_goal_handle = None

    def _is_stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def _run_parking_mission(self) -> None:
        self._publish_status("STEP1_PARKING")
        self.get_logger().info("Step1 parking started.")
        self._publish_status("STEP1_CHECK_CUSHION_SIDE")
        robot_pose = self._get_fresh_pose()
        cushion_pose = self._get_fresh_cushion_pose()
        if robot_pose is None or cushion_pose is None:
            self._publish_status("MISSION_FAILED")
            self.get_logger().error("Parking failed: stale robot or cushion pose")
            return

        geometry = CushionGeometry(
            center_x=cushion_pose[0],
            center_y=cushion_pose[1],
            length=self.cushion_length,
            width=self.cushion_width,
            yaw=cushion_pose[2],
            front_axis=self.parking_front_axis,
            front_normal_sign=self.front_normal_sign,
        )
        config = ParkingPlannerConfig(
            front_side_threshold=self.front_side_threshold,
            side_clearance=self.side_clearance,
            front_clearance=self.front_clearance,
            desired_normal_distance=self.desired_normal_distance,
            tangential_offset=self.tangential_offset,
            parking_lateral_offset=self.parking_lateral_offset,
            pre_park_backoff=self.pre_park_backoff,
            robot_safety_radius=self.parking_robot_safety_radius,
            stick_safety_extension=self.stick_safety_extension,
            safety_margin=self.parking_safety_margin,
            circle_spacing=self.cushion_circle_spacing,
            obstacle_axis=self.cushion_obstacle_axis,
            obstacle_radius_override=self.cushion_obstacle_radius_override,
        )
        plan = plan_parking_route(
            (robot_pose[0], robot_pose[1]),
            geometry,
            config,
        )
        self.get_logger().info(
            "Parking obstacle model: "
            f"axis={self.cushion_obstacle_axis}, "
            f"centers={[ (round(o.x, 3), round(o.y, 3)) for o in plan.cushion_obstacles ]}, "
            f"radii={[ round(o.radius, 3) for o in plan.cushion_obstacles ]}"
        )
        self._publish_parking_markers(robot_pose, geometry, config, plan)
        if not plan.waypoints:
            self._publish_status("FAILED")
            self.get_logger().error(f"Parking planning failed: {plan.message}")
            return

        if not self._configure_safe_navigation_for_parking(plan):
            self._publish_status("FAILED")
            return

        if not plan.already_front_side:
            self._publish_status("STEP1_SELECT_BYPASS_SIDE")
            self.get_logger().info(f"Selected {plan.bypass_side} bypass route")
            for waypoint in plan.waypoints[:-2]:
                self._publish_status("STEP1_GO_TO_BYPASS_WAYPOINT")
                success, message = self._run_safe_navigation_to_point(
                    waypoint[0],
                    waypoint[1],
                    self.linear_speed,
                    self.safe_navigation_timeout_sec,
                )
                if self._parking_step_failed(success, message):
                    return
        else:
            self.get_logger().info("Robot already on cushion parking side")

        self._publish_status("STEP1_GO_TO_PRE_PARK_POINT")
        success, message = self._run_safe_navigation_to_point(
            plan.pre_park_point[0],
            plan.pre_park_point[1],
            self.linear_speed,
            self.safe_navigation_timeout_sec,
        )
        if self._parking_step_failed(success, message):
            return

        self._publish_status("STEP1_ALIGN_FOR_FINAL_APPROACH")
        if not self._align_to_yaw(plan.final_yaw):
            self._publish_status("FAILED")
            return

        self._publish_status("STEP1_FINAL_APPROACH")
        self._set_safe_nav_parameters(
            {
                "point_gain": self.final_approach_point_gain,
            }
        )
        final_point = self._control_point_goal_for_robot_pose(
            plan.final_park_point,
            plan.final_yaw,
        )
        success, message = self._run_safe_navigation_to_point(
            final_point[0],
            final_point[1],
            self.final_approach_speed,
            self.safe_navigation_timeout_sec,
        )
        if self._parking_step_failed(success, message):
            return

        if not self._align_to_yaw(plan.final_yaw):
            self._publish_status("FAILED")
            return

        self._publish_status("STEP1_PARKING_DONE")
        self.get_logger().info(
            "Step1 parking succeeded. Transitioning to step2 stick pickup."
        )
        self._publish_status("STEP2_PICK_UP_STICK")

        stick_success, stick_message = self._run_stick_setup_mission()
        if self._is_stop_requested():
            self._publish_status("MISSION_STOPPED")
            return
        if not stick_success:
            self._publish_status("STEP2_PICK_UP_STICK_FAILED")
            self.get_logger().error(f"Step2 stick pickup failed: {stick_message}")
            return

        self._publish_status("MISSION_DONE")
        self.get_logger().info("Mission completed successfully: parking and stick pickup finished.")

    def _run_stick_setup_mission(self) -> Tuple[bool, str]:
        """Pick up the hockey stick and place the arm in its ready pose."""
        if not self.stick_setup_enabled:
            self._publish_status("STEP2_PICK_UP_STICK_SKIPPED")
            self.get_logger().warning(
                "Step2 stick pickup is disabled; set stick_setup_enabled=true "
                "after calibrating the arm positions."
            )
            return True, "Stick setup disabled"

        success, message = self._run_arm_step(
            "STEP2_MOVE_ARM_TO_GRAB",
            self.grab_arm_x,
            self.grab_arm_z,
            self.grab_arm_relative,
            self.grab_arm_timeout_sec,
            self.grab_arm_settle_sec,
        )
        if not success:
            return False, f"Move arm to grab position: {message}"

        success, message = self._run_gripper_close_step()
        if not success:
            return False, f"Close gripper: {message}"

        success, message = self._run_arm_step(
            "STEP2_LIFT_HOCKEY_STICK",
            self.lift_arm_x,
            self.lift_arm_z,
            self.lift_arm_relative,
            self.lift_arm_timeout_sec,
            self.lift_arm_settle_sec,
        )
        if not success:
            return False, f"Lift hockey stick: {message}"

        success, message = self._run_backward_step()
        if not success:
            return False, f"Back up with hockey stick: {message}"

        success, message = self._run_arm_step(
            "STEP2_LOWER_ARM_TO_READY",
            self.ready_arm_x,
            self.ready_arm_z,
            self.ready_arm_relative,
            self.ready_arm_timeout_sec,
            self.ready_arm_settle_sec,
        )
        if not success:
            return False, f"Lower arm to ready position: {message}"

        self._publish_status("STEP2_STICK_READY")
        self.get_logger().info("Hockey stick is in the ready-to-hit position.")
        return True, "Hockey stick setup completed"

    def _run_arm_step(
        self,
        status: str,
        x: float,
        z: float,
        relative: bool,
        timeout_sec: float,
        settle_sec: float,
    ) -> Tuple[bool, str]:
        self._publish_status(status)
        if self._is_stop_requested():
            return False, "Mission stop requested"
        if self._arm_client is None or MoveArm is None:
            return False, "MoveArm action type is unavailable"
        if not self._arm_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.arm_action}"

        goal = MoveArm.Goal()
        goal.x = float(x)
        goal.z = float(z)
        goal.relative = bool(relative)
        success, message = self._send_goal_and_wait(
            self._arm_client,
            goal,
            self._handle_arm_feedback,
            timeout_sec=timeout_sec,
        )
        if not success:
            return False, message
        if not self._interruptible_wait(settle_sec):
            return False, "Mission stop requested during arm settling"
        return True, message

    def _run_gripper_close_step(self) -> Tuple[bool, str]:
        self._publish_status("STEP2_CLOSE_GRIPPER")
        if self._is_stop_requested():
            return False, "Mission stop requested"
        if self._gripper_client is None or GripperControl is None:
            return False, "GripperControl action type is unavailable"
        if not self._gripper_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.gripper_action}"

        goal = GripperControl.Goal()
        goal.target_state = GripperControl.Goal.CLOSE
        goal.power = float(self.gripper_close_power)
        success, message = self._send_goal_and_wait(
            self._gripper_client,
            goal,
            self._handle_gripper_feedback,
            timeout_sec=self.gripper_close_timeout_sec,
        )
        if not success:
            return False, message
        if not self._interruptible_wait(self.gripper_close_settle_sec):
            return False, "Mission stop requested while gripper was settling"
        return True, message

    def _run_backward_step(self) -> Tuple[bool, str]:
        self._publish_status("STEP2_BACK_UP_WITH_STICK")
        if self._is_stop_requested():
            return False, "Mission stop requested"

        backward_speed = -self.backward_distance / self.backward_duration_sec
        period_sec = 1.0 / self.backward_publish_rate_hz
        twist = Twist()
        twist.linear.x = backward_speed
        start_time = monotonic()

        self.get_logger().info(
            "Backing up with hockey stick: "
            f"distance={self.backward_distance:.3f}, "
            f"duration={self.backward_duration_sec:.3f}, "
            f"speed={backward_speed:.3f}"
        )
        try:
            while (
                rclpy.ok()
                and monotonic() - start_time < self.backward_duration_sec
            ):
                if self._is_stop_requested():
                    return (
                        False,
                        "Mission stop requested during reverse motion",
                    )
                self._cmd_vel_publisher.publish(twist)
                Event().wait(period_sec)
        finally:
            self._stop_robot()

        if not rclpy.ok():
            return False, "ROS shutdown interrupted reverse motion"
        return True, "Reverse motion completed"

    def _interruptible_wait(self, duration_sec: float) -> bool:
        end_time = monotonic() + duration_sec
        while rclpy.ok() and monotonic() < end_time:
            if self._is_stop_requested():
                return False
            Event().wait(min(0.05, max(0.0, end_time - monotonic())))
        return rclpy.ok() and not self._is_stop_requested()

    def _control_point_goal_for_robot_pose(
        self,
        robot_position: Tuple[float, float],
        robot_yaw: float,
    ) -> Tuple[float, float]:
        return (
            robot_position[0]
            + self.parking_lookahead_distance * math.cos(robot_yaw),
            robot_position[1]
            + self.parking_lookahead_distance * math.sin(robot_yaw),
        )

    def _publish_parking_markers(
        self,
        robot_pose: Tuple[float, float, float],
        geometry: CushionGeometry,
        config: ParkingPlannerConfig,
        plan,
    ) -> None:
        if Marker is None or MarkerArray is None:
            self.get_logger().warning(
                "visualization_msgs is unavailable; parking markers disabled"
            )
            return
        if self._parking_marker_publisher is None:
            return

        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        marker_id = 0
        markers.markers.append(
            self._cube_marker(
                marker_id,
                "cushion_body",
                geometry.center_x,
                geometry.center_y,
                geometry.yaw,
                geometry.length,
                geometry.width,
                0.04,
                ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.45),
            )
        )
        marker_id += 1

        radius_layers = self._parking_radius_layers(geometry, config)
        for obstacle in plan.cushion_obstacles:
            for namespace, radius, color in radius_layers:
                markers.markers.append(
                    self._circle_marker(
                        marker_id,
                        namespace,
                        obstacle.x,
                        obstacle.y,
                        radius,
                        color,
                    )
                )
                marker_id += 1

        markers.markers.append(
            self._circle_marker(
                marker_id,
                "robot_safety_radius",
                robot_pose[0],
                robot_pose[1],
                config.robot_safety_radius,
                ColorRGBA(r=0.0, g=0.8, b=0.25, a=0.28),
            )
        )
        marker_id += 1

        t, n = cushion_axes(geometry)
        markers.markers.append(
            self._arrow_marker(
                marker_id,
                "front_normal",
                (geometry.center_x, geometry.center_y),
                (
                    geometry.center_x + 0.4 * n[0],
                    geometry.center_y + 0.4 * n[1],
                ),
                ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.9),
            )
        )
        marker_id += 1
        markers.markers.append(
            self._arrow_marker(
                marker_id,
                "cushion_local_x",
                (geometry.center_x, geometry.center_y),
                (
                    geometry.center_x + 0.4 * t[0],
                    geometry.center_y + 0.4 * t[1],
                ),
                ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9),
            )
        )
        marker_id += 1

        route_points = [(robot_pose[0], robot_pose[1])]
        route_points.extend(plan.waypoints)
        if len(route_points) >= 2:
            markers.markers.append(
                self._line_strip_marker(
                    marker_id,
                    "parking_route",
                    route_points,
                    ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9),
                )
            )
            marker_id += 1

        labeled_points = [
            ("pre_park_point", plan.pre_park_point, ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.95)),
            ("final_park_point", plan.final_park_point, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.95)),
        ]
        if not plan.already_front_side and len(plan.waypoints) > 2:
            for index, waypoint in enumerate(reversed(plan.waypoints[:-2])):
                labeled_points.insert(
                    0,
                    (
                        f"bypass_waypoint_{len(plan.waypoints[:-2]) - index}",
                        waypoint,
                        ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.95),
                    ),
                )
        for namespace, point, color in labeled_points:
            markers.markers.append(
                self._sphere_marker(
                    marker_id,
                    namespace,
                    point[0],
                    point[1],
                    0.08,
                    color,
                )
            )
            marker_id += 1

        final_heading_end = (
            plan.final_park_point[0] + 0.25 * math.cos(plan.final_yaw),
            plan.final_park_point[1] + 0.25 * math.sin(plan.final_yaw),
        )
        markers.markers.append(
            self._arrow_marker(
                marker_id,
                "final_yaw",
                plan.final_park_point,
                final_heading_end,
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
            )
        )

        self._parking_marker_publisher.publish(markers)

    def _parking_radius_layers(
        self,
        geometry: CushionGeometry,
        config: ParkingPlannerConfig,
    ):
        physical, robot, stick, cbf = cushion_radius_layers(geometry, config)
        layers = [
            (
                "cushion_cbf_radius",
                cbf,
                ColorRGBA(r=0.1, g=0.35, b=1.0, a=0.16),
            )
        ]
        if stick > robot + 1e-9:
            layers.append(
                (
                    "cushion_stick_inflated_radius",
                    stick,
                    ColorRGBA(r=0.45, g=0.15, b=1.0, a=0.18),
                )
            )
        layers.extend(
            [
                (
                    "cushion_robot_inflated_radius",
                    robot,
                    ColorRGBA(r=0.0, g=0.75, b=1.0, a=0.20),
                ),
                (
                    "cushion_physical_radius",
                    physical,
                    ColorRGBA(r=0.9, g=0.55, b=0.05, a=0.26),
                ),
            ]
        )
        unique_layers = []
        previous_radius = None
        for namespace, radius, color in layers:
            if previous_radius is not None and abs(radius - previous_radius) < 1e-9:
                continue
            unique_layers.append((namespace, radius, color))
            previous_radius = radius
        return unique_layers

    def _base_marker(self, marker_id: int, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.visualization_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _circle_marker(
        self,
        marker_id: int,
        namespace: str,
        x: float,
        y: float,
        radius: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.CYLINDER)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.01
        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = 0.02
        marker.color = color
        return marker

    def _sphere_marker(
        self,
        marker_id: int,
        namespace: str,
        x: float,
        y: float,
        diameter: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.SPHERE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = color
        return marker

    def _cube_marker(
        self,
        marker_id: int,
        namespace: str,
        x: float,
        y: float,
        yaw: float,
        length: float,
        width: float,
        height: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.CUBE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.03
        marker.pose.orientation.z = math.sin(0.5 * yaw)
        marker.pose.orientation.w = math.cos(0.5 * yaw)
        marker.scale.x = length
        marker.scale.y = width
        marker.scale.z = height
        marker.color = color
        return marker

    def _arrow_marker(
        self,
        marker_id: int,
        namespace: str,
        start: Tuple[float, float],
        end: Tuple[float, float],
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.ARROW)
        start_point = Point()
        start_point.x = start[0]
        start_point.y = start[1]
        start_point.z = 0.08
        end_point = Point()
        end_point.x = end[0]
        end_point.y = end[1]
        end_point.z = 0.08
        marker.points.append(start_point)
        marker.points.append(end_point)
        marker.scale.x = 0.025
        marker.scale.y = 0.055
        marker.scale.z = 0.08
        marker.color = color
        return marker

    def _line_strip_marker(
        self,
        marker_id: int,
        namespace: str,
        points: list,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = 0.025
        marker.color = color
        for x, y in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.06
            marker.points.append(point)
        return marker

    def _configure_safe_navigation_for_parking(
        self,
        plan,
    ) -> bool:
        obstacle_x = [obstacle.x for obstacle in plan.cushion_obstacles]
        obstacle_y = [obstacle.y for obstacle in plan.cushion_obstacles]
        obstacle_radius = [obstacle.radius for obstacle in plan.cushion_obstacles]
        return self._set_safe_nav_parameters(
            {
                "use_target_pose": False,
                "orient_to_target": False,
                "obstacles_enabled": True,
                "robot_safety_radius": 0.0,
                "obstacle_safe_margin": 0.0,
                "obstacle_x": obstacle_x,
                "obstacle_y": obstacle_y,
                "obstacle_radius": obstacle_radius,
            }
        )

    def _parking_step_failed(self, success: bool, message: str) -> bool:
        if self._is_stop_requested():
            self._publish_status("MISSION_STOPPED")
            return True
        if success:
            return False
        self._publish_status("FAILED")
        self.get_logger().error(f"Parking step failed: {message}")
        return True

    def _run_safe_navigation_to_point(
        self,
        target_x: float,
        target_y: float,
        linear_speed: float,
        timeout_sec: float,
    ) -> Tuple[bool, str]:
        if not self._safe_navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return (
                False,
                f"Action server unavailable: {self.safe_navigation_action}",
            )

        goal = NavigateToPoint.Goal()
        goal.target_x = float(target_x)
        goal.target_y = float(target_y)
        goal.linear_speed = float(linear_speed)
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = float(timeout_sec)

        return self._send_goal_and_wait(
            self._safe_navigation_client,
            goal,
            self._handle_safe_navigation_feedback,
        )

    def _align_to_yaw(self, target_yaw: float) -> bool:
        start_time = self.get_clock().now()
        rate_sec = 0.05
        last_pose = None
        last_yaw_error = None
        while rclpy.ok() and not self._is_stop_requested():
            pose = self._get_fresh_pose()
            if pose is None:
                self._stop_robot()
                self.get_logger().error("Align failed: stale robot pose")
                return False
            yaw_error = wrap_to_pi(target_yaw - pose[2])
            last_pose = pose
            last_yaw_error = yaw_error
            if abs(yaw_error) <= self.final_yaw_tolerance:
                self._stop_robot()
                return True
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.align_timeout_sec:
                self._stop_robot()
                if last_pose is None or last_yaw_error is None:
                    self.get_logger().error("Align failed: timeout")
                else:
                    self.get_logger().error(
                        "Align failed: timeout "
                        f"target_yaw={target_yaw:.3f}, "
                        f"current_yaw={last_pose[2]:.3f}, "
                        f"yaw_error={last_yaw_error:.3f}, "
                        f"tolerance={self.final_yaw_tolerance:.3f}, "
                        f"cmd_vel={self.cmd_vel_topic}"
                    )
                return False
            twist = Twist()
            twist.angular.z = clamp(
                self.align_gain * yaw_error,
                -self.angular_speed,
                self.angular_speed,
            )
            self._cmd_vel_publisher.publish(twist)
            Event().wait(rate_sec)
        self._stop_robot()
        return False

    def _set_safe_nav_parameters(self, values) -> bool:
        if not self._safe_nav_parameter_client.wait_for_service(
            timeout_sec=self.action_wait_timeout_sec
        ):
            self.get_logger().error("safe_navigation_server parameter service unavailable")
            return False

        request = SetParameters.Request()
        request.parameters = [
            self._parameter_message(name, value)
            for name, value in values.items()
        ]
        future = self._safe_nav_parameter_client.call_async(request)
        while rclpy.ok() and not future.done():
            if self._is_stop_requested():
                return False
            Event().wait(0.05)
        response = future.result()
        for result in response.results:
            if not result.successful:
                self.get_logger().error(
                    f"Failed to configure safe nav parameter: {result.reason}"
                )
                return False
        return True

    def _parameter_message(self, name: str, value) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, str):
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter.value.double_array_value = [float(item) for item in value]
        return parameter

    def _run_navigation_step(self) -> Tuple[bool, str]:
        if not self._navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.navigation_action}"

        goal = NavigateToPoint.Goal()
        goal.target_x = self.target_x
        goal.target_y = self.target_y
        goal.linear_speed = self.linear_speed
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.navigation_timeout_sec

        return self._send_goal_and_wait(
            self._navigation_client,
            goal,
            self._handle_navigation_feedback,
        )

    def _run_safe_navigation_step(self) -> Tuple[bool, str]:
        if not self._safe_navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return (
                False,
                f"Action server unavailable: {self.safe_navigation_action}",
            )

        goal = NavigateToPoint.Goal()
        goal.target_x = self.safe_target_x
        goal.target_y = self.safe_target_y
        goal.linear_speed = self.linear_speed
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.safe_navigation_timeout_sec

        return self._send_goal_and_wait(
            self._safe_navigation_client,
            goal,
            self._handle_safe_navigation_feedback,
        )

    def _run_spin_step(self) -> Tuple[bool, str]:
        if not self._spin_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.spin_action}"

        goal = Spin.Goal()
        goal.rotations = self.rotations
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.spin_timeout_sec

        return self._send_goal_and_wait(
            self._spin_client,
            goal,
            self._handle_spin_feedback,
        )

    def _send_goal_and_wait(
        self,
        client,
        goal,
        feedback_callback,
        timeout_sec: Optional[float] = None,
    ) -> Tuple[bool, str]:
        done_event = Event()
        abandoned_event = Event()
        goal_handle_holder = {"handle": None}
        result_holder = {
            "success": False,
            "message": "Action did not finish",
        }

        def handle_goal_response(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exception:
                result_holder["message"] = f"Goal request failed: {exception}"
                done_event.set()
                return
            if not goal_handle.accepted:
                result_holder["message"] = "Goal rejected"
                done_event.set()
                return

            if abandoned_event.is_set():
                goal_handle.cancel_goal_async()
                return

            goal_handle_holder["handle"] = goal_handle
            with self._lock:
                self._active_goal_handle = goal_handle

            if self._is_stop_requested():
                goal_handle.cancel_goal_async()

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(handle_result)

        def handle_result(future) -> None:
            try:
                result_wrapper = future.result()
                result = result_wrapper.result
                result_holder["success"] = bool(result.success)
                result_holder["message"] = str(result.message)
            except Exception as exception:
                result_holder["message"] = f"Action result failed: {exception}"
            with self._lock:
                if self._active_goal_handle is goal_handle_holder["handle"]:
                    self._active_goal_handle = None
            done_event.set()

        try:
            send_future = client.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            )
        except Exception as exception:
            return False, f"Failed to send action goal: {exception}"
        send_future.add_done_callback(handle_goal_response)
        start_time = monotonic()
        while not done_event.wait(timeout=0.05):
            if self._is_stop_requested():
                abandoned_event.set()
                with self._lock:
                    active_goal_handle = self._active_goal_handle
                    self._active_goal_handle = None
                if active_goal_handle is not None:
                    active_goal_handle.cancel_goal_async()
                return False, "Mission stop requested"

            if (
                timeout_sec is not None
                and monotonic() - start_time >= timeout_sec
            ):
                abandoned_event.set()
                with self._lock:
                    active_goal_handle = self._active_goal_handle
                    self._active_goal_handle = None
                if active_goal_handle is not None:
                    active_goal_handle.cancel_goal_async()
                return (
                    False,
                    f"Action timed out after {timeout_sec:.2f} seconds",
                )

        return bool(result_holder["success"]), str(result_holder["message"])

    def _handle_navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "Step1 feedback: "
            f"{feedback.state}, "
            f"distance={feedback.distance_remaining:.2f}"
        )

    def _handle_safe_navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "Step1 feedback: "
            f"{feedback.state}, "
            f"distance={feedback.distance_remaining:.2f}"
        )

    def _handle_spin_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "Step2 feedback: "
            f"{feedback.state}, "
            f"rotation_remaining={feedback.rotation_remaining:.2f}"
        )

    def _handle_arm_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Arm action progress={feedback.progress:.2f}"
        )

    def _handle_gripper_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Gripper current state={feedback.current_state}"
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

    def _cushion_pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        with self._cushion_pose_lock:
            self._latest_cushion_pose = (
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
            self._latest_cushion_pose_time = self.get_clock().now()

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

    def _get_fresh_cushion_pose(self) -> Optional[Tuple[float, float, float]]:
        with self._cushion_pose_lock:
            pose = self._latest_cushion_pose
            pose_time = self._latest_cushion_pose_time
        if pose is None or pose_time is None:
            return None
        pose_age = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        if pose_age > self.pose_timeout_sec:
            return None
        return pose

    def _stop_robot(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
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
