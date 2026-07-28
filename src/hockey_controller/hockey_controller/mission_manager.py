#!/usr/bin/env python3

import math
from threading import Event, Lock, Thread
from time import monotonic
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

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
from hockey_controller.cushion_parking_planner import plan_parking_route
from hockey_controller.navigation_server import clamp
from hockey_controller.navigation_server import wrap_to_pi
from hockey_controller.navigation_server import yaw_from_quaternion
from hockey_controller.parking_markers import marker_array_type
from hockey_controller.parking_markers import publish_parking_markers
from hockey_controller.parking_markers import visualization_available


class MissionManager(Node):
    """Coordinate parking, optional stick pickup, and fallback mission steps."""

    PARAMETER_DEFAULTS = {
        "navigation_action": "navigate_to_point",
        "safe_navigation_action": "safe_navigate_to_point",
        "spin_action": "spin",
        "arm_action": "control_arm",
        "gripper_action": "control_gripper",
        "robot_id": 1,
        "pose_topic": "",
        "cushion_pose_topic": "",
        "parking_enabled": True,
        "target_x": 1.0,
        "target_y": 0.0,
        "safe_target_x": 1.0,
        "safe_target_y": 0.0,
        "cushion_length": 1.0,
        "cushion_width": 0.12,
        "parking_front_axis": "y",
        "front_normal_sign": -1.0,
        "front_side_threshold": 0.0,
        "side_clearance": 0.35,
        "front_clearance": 0.35,
        "desired_normal_distance": 0.35,
        "tangential_offset": 0.0,
        "parking_lateral_offset": 0.0,
        "pre_park_backoff": 0.40,
        "parking_robot_safety_radius": 0.20,
        "stick_safety_extension": 0.0,
        "parking_safety_margin": 0.10,
        "cushion_circle_spacing": 0.20,
        "cushion_obstacle_axis": "local_x",
        "cushion_obstacle_radius_override": -1.0,
        "parking_lookahead_distance": 0.25,
        "final_approach_speed": 0.12,
        "final_approach_point_gain": 0.35,
        "align_gain": 2.0,
        "align_timeout_sec": 8.0,
        "final_yaw_tolerance": 0.08,
        "pose_timeout_sec": 1.0,
        "visualization_frame": "map",
        "rotations": 1,
        "linear_speed": 0.4,
        "angular_speed": 0.8,
        "navigation_timeout_sec": 30.0,
        "safe_navigation_timeout_sec": 30.0,
        "spin_timeout_sec": 15.0,
        "action_wait_timeout_sec": 5.0,
        "stick_setup_enabled": False,
        "grab_arm_x": 0.0,
        "grab_arm_z": 0.0,
        "grab_arm_relative": False,
        "grab_arm_timeout_sec": 8.0,
        "grab_arm_settle_sec": 0.3,
        "gripper_close_power": 0.5,
        "gripper_close_timeout_sec": 5.0,
        "gripper_close_settle_sec": 0.5,
        "lift_arm_x": 1.0,
        "lift_arm_z": 2.0,
        "lift_arm_relative": False,
        "lift_arm_timeout_sec": 8.0,
        "lift_arm_settle_sec": 0.3,
        "backward_distance": 0.30,
        "backward_duration_sec": 2.0,
        "backward_publish_rate_hz": 20.0,
        "backward_max_speed": 0.30,
        "ready_arm_x": 0.0,
        "ready_arm_z": 0.0,
        "ready_arm_relative": False,
        "ready_arm_timeout_sec": 8.0,
        "ready_arm_settle_sec": 0.3,
    }

    def __init__(self) -> None:
        super().__init__("mission_manager")
        for name, default in self.PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)

        self._callback_group = ReentrantCallbackGroup()
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

        self._reload_parameters()
        self.pose_topic = self._resolved_pose_topic()
        self.cushion_pose_topic = self._resolved_cushion_pose_topic()
        self.cmd_vel_topic = f"/robot{self.robot_id}/cmd_vel"

        self._status_publisher = self.create_publisher(String, "mission/status", 10)
        self._cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._parking_marker_publisher = None
        if visualization_available():
            self._parking_marker_publisher = self.create_publisher(
                marker_array_type(),
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
        self._arm_client = (
            ActionClient(
                self,
                MoveArm,
                self.arm_action,
                callback_group=self._callback_group,
            )
            if MoveArm is not None
            else None
        )
        self._gripper_client = (
            ActionClient(
                self,
                GripperControl,
                self.gripper_action,
                callback_group=self._callback_group,
            )
            if GripperControl is not None
            else None
        )
        self._safe_nav_parameter_client = self.create_client(
            SetParameters,
            "safe_navigation_server/set_parameters",
            callback_group=self._callback_group,
        )

        self._publish_status("IDLE")
        self.get_logger().info(
            "Mission manager ready. Call mission/start in this node namespace.\n"
            f"  pose        = {self.pose_topic}\n"
            f"  cushion     = {self.cushion_pose_topic}\n"
            f"  safe action = {self.safe_navigation_action}\n"
            f"  parking     = {self.parking_enabled}\n"
            f"  stick setup = {self.stick_setup_enabled}"
        )

    def _handle_parameter_update(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "cushion_pose_topic":
                topic = str(parameter.value) or "/vrpn_mocap/hockey_sticks_1/pose"
                self._reset_cushion_pose_subscription(topic)
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
        self.get_logger().info(f"Updated cushion pose subscription: {topic}")

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
            valid, message = self._validate_stick_setup_parameters()
            if not valid:
                response.success = False
                response.message = message
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
        for name, default in self.PARAMETER_DEFAULTS.items():
            value = self.get_parameter(name).value
            if isinstance(default, bool):
                value = bool(value)
            elif isinstance(default, int) and not isinstance(default, bool):
                value = int(value)
            elif isinstance(default, float):
                value = float(value)
            elif isinstance(default, str):
                value = str(value)
            setattr(self, name, value)

    def _resolved_pose_topic(self) -> str:
        return str(self.pose_topic) or f"/vrpn_mocap/dji_robot_{self.robot_id}/pose"

    def _resolved_cushion_pose_topic(self) -> str:
        return str(self.cushion_pose_topic) or "/vrpn_mocap/hockey_sticks_1/pose"

    def _validate_stick_setup_parameters(self) -> Tuple[bool, str]:
        if not self.stick_setup_enabled:
            return True, "Stick setup is disabled"
        if MoveArm is None or GripperControl is None:
            return (
                False,
                "Stick setup requires MoveArm and GripperControl actions. "
                "Rebuild hockey_interfaces first.",
            )
        if not 0.0 <= self.gripper_close_power <= 1.0:
            return False, "gripper_close_power must be in [0, 1]"
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
                return False, f"{name} must be > 0"
        speed = self.backward_distance / self.backward_duration_sec
        if speed > self.backward_max_speed:
            return False, "backward_distance / backward_duration_sec exceeds backward_max_speed"
        return True, "Stick setup parameters are valid"

    def _run_mission(self) -> None:
        try:
            if self.parking_enabled:
                self._run_parking_mission()
            else:
                self._run_navigation_and_spin_mission()
        except Exception as exception:
            self._publish_status("MISSION_FAILED")
            self.get_logger().error(f"Mission exception: {exception}")
        finally:
            self._stop_robot()
            with self._lock:
                self._running = False
                self._stop_requested = False
                self._active_goal_handle = None

    def _run_parking_mission(self) -> None:
        self._publish_status("STEP1_PARKING")
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
        plan = plan_parking_route((robot_pose[0], robot_pose[1]), geometry, config)
        self._publish_parking_plan(robot_pose, geometry, config, plan)
        if not plan.waypoints:
            self._publish_status("MISSION_FAILED")
            self.get_logger().error(f"Parking planning failed: {plan.message}")
            return
        if not self._configure_safe_navigation_for_parking(plan):
            self._publish_status("MISSION_FAILED")
            return

        if not plan.already_front_side:
            self.get_logger().info(f"Selected {plan.bypass_side} bypass route")
            for waypoint in plan.waypoints[:-2]:
                self._publish_status("STEP1_GO_TO_BYPASS_WAYPOINT")
                if self._parking_step_failed(
                    self._navigate_safe(waypoint, self.linear_speed)
                ):
                    return
        else:
            self.get_logger().info("Robot already on cushion parking side")

        self._publish_status("STEP1_GO_TO_PRE_PARK_POINT")
        if self._parking_step_failed(
            self._navigate_safe(plan.pre_park_point, self.linear_speed)
        ):
            return

        self._publish_status("STEP1_ALIGN_FOR_FINAL_APPROACH")
        if not self._align_to_yaw(plan.final_yaw):
            self._publish_status("MISSION_FAILED")
            return

        self._publish_status("STEP1_FINAL_APPROACH")
        self._set_safe_nav_parameters({"point_gain": self.final_approach_point_gain})
        final_goal = self._control_point_goal(plan.final_park_point, plan.final_yaw)
        if self._parking_step_failed(
            self._navigate_safe(final_goal, self.final_approach_speed)
        ):
            return
        if not self._align_to_yaw(plan.final_yaw):
            self._publish_status("MISSION_FAILED")
            return

        self._publish_status("STEP1_PARKING_DONE")
        stick_success, stick_message = self._run_stick_setup_mission()
        if self._is_stop_requested():
            self._publish_status("MISSION_STOPPED")
            return
        if not stick_success:
            self._publish_status("MISSION_FAILED")
            self.get_logger().error(f"Stick setup failed: {stick_message}")
            return
        self._publish_status("MISSION_DONE")
        self.get_logger().info("Mission completed successfully.")

    def _publish_parking_plan(self, robot_pose, geometry, config, plan) -> None:
        self.get_logger().info(
            "Parking obstacle model: "
            f"axis={self.cushion_obstacle_axis}, "
            f"centers={[ (round(o.x, 3), round(o.y, 3)) for o in plan.cushion_obstacles ]}, "
            f"radii={[ round(o.radius, 3) for o in plan.cushion_obstacles ]}"
        )
        publish_parking_markers(
            self,
            self._parking_marker_publisher,
            self.visualization_frame,
            robot_pose,
            geometry,
            config,
            plan,
        )

    def _run_navigation_and_spin_mission(self) -> None:
        success, message = self._run_navigation_step()
        if self._parking_step_failed((success, message)):
            return
        success, message = self._run_spin_step()
        if self._parking_step_failed((success, message)):
            return
        self._publish_status("MISSION_DONE")

    def _run_stick_setup_mission(self) -> Tuple[bool, str]:
        if not self.stick_setup_enabled:
            self._publish_status("STEP2_PICK_UP_STICK_SKIPPED")
            self.get_logger().warning("Stick setup disabled.")
            return True, "Stick setup disabled"

        steps = (
            lambda: self._run_arm_step(
                "STEP2_MOVE_ARM_TO_GRAB",
                self.grab_arm_x,
                self.grab_arm_z,
                self.grab_arm_relative,
                self.grab_arm_timeout_sec,
                self.grab_arm_settle_sec,
            ),
            self._run_gripper_close_step,
            lambda: self._run_arm_step(
                "STEP2_LIFT_HOCKEY_STICK",
                self.lift_arm_x,
                self.lift_arm_z,
                self.lift_arm_relative,
                self.lift_arm_timeout_sec,
                self.lift_arm_settle_sec,
            ),
            self._run_backward_step,
            lambda: self._run_arm_step(
                "STEP2_LOWER_ARM_TO_READY",
                self.ready_arm_x,
                self.ready_arm_z,
                self.ready_arm_relative,
                self.ready_arm_timeout_sec,
                self.ready_arm_settle_sec,
            ),
        )
        for step in steps:
            success, message = step()
            if not success:
                return False, message
        self._publish_status("STEP2_STICK_READY")
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
        if not self._arm_client.wait_for_server(timeout_sec=self.action_wait_timeout_sec):
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
        if not self._gripper_client.wait_for_server(timeout_sec=self.action_wait_timeout_sec):
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
        speed = -self.backward_distance / self.backward_duration_sec
        period_sec = 1.0 / self.backward_publish_rate_hz
        twist = Twist()
        twist.linear.x = speed
        start_time = monotonic()
        try:
            while rclpy.ok() and monotonic() - start_time < self.backward_duration_sec:
                if self._is_stop_requested():
                    return False, "Mission stop requested during reverse motion"
                self._cmd_vel_publisher.publish(twist)
                Event().wait(period_sec)
        finally:
            self._stop_robot()
        return True, "Reverse motion completed"

    def _interruptible_wait(self, duration_sec: float) -> bool:
        end_time = monotonic() + duration_sec
        while rclpy.ok() and monotonic() < end_time:
            if self._is_stop_requested():
                return False
            Event().wait(min(0.05, max(0.0, end_time - monotonic())))
        return rclpy.ok() and not self._is_stop_requested()

    def _configure_safe_navigation_for_parking(self, plan) -> bool:
        return self._set_safe_nav_parameters(
            {
                "use_target_pose": False,
                "orient_to_target": False,
                "obstacles_enabled": True,
                "robot_safety_radius": 0.0,
                "obstacle_safe_margin": 0.0,
                "obstacle_x": [obstacle.x for obstacle in plan.cushion_obstacles],
                "obstacle_y": [obstacle.y for obstacle in plan.cushion_obstacles],
                "obstacle_radius": [
                    obstacle.radius for obstacle in plan.cushion_obstacles
                ],
            }
        )

    def _navigate_safe(self, point, speed: float) -> Tuple[bool, str]:
        goal = NavigateToPoint.Goal()
        goal.target_x = float(point[0])
        goal.target_y = float(point[1])
        goal.linear_speed = float(speed)
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.safe_navigation_timeout_sec
        if not self._safe_navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.safe_navigation_action}"
        return self._send_goal_and_wait(
            self._safe_navigation_client,
            goal,
            self._handle_safe_navigation_feedback,
        )

    def _parking_step_failed(self, result: Tuple[bool, str]) -> bool:
        success, message = result
        if self._is_stop_requested():
            self._publish_status("MISSION_STOPPED")
            return True
        if success:
            return False
        self._publish_status("MISSION_FAILED")
        self.get_logger().error(f"Parking step failed: {message}")
        return True

    def _align_to_yaw(self, target_yaw: float) -> bool:
        start_time = self.get_clock().now()
        while rclpy.ok() and not self._is_stop_requested():
            pose = self._get_fresh_pose()
            if pose is None:
                self._stop_robot()
                self.get_logger().error("Align failed: stale robot pose")
                return False
            yaw_error = wrap_to_pi(target_yaw - pose[2])
            if abs(yaw_error) <= self.final_yaw_tolerance:
                self._stop_robot()
                return True
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.align_timeout_sec:
                self._stop_robot()
                self.get_logger().error(
                    "Align failed: timeout "
                    f"target_yaw={target_yaw:.3f}, "
                    f"current_yaw={pose[2]:.3f}, "
                    f"yaw_error={yaw_error:.3f}, "
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
            Event().wait(0.05)
        self._stop_robot()
        return False

    def _control_point_goal(self, position, yaw: float) -> Tuple[float, float]:
        return (
            position[0] + self.parking_lookahead_distance * math.cos(yaw),
            position[1] + self.parking_lookahead_distance * math.sin(yaw),
        )

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
        elif isinstance(value, int) and not isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        elif isinstance(value, str):
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter.value.double_array_value = [float(item) for item in value]
        return parameter

    def _run_navigation_step(self) -> Tuple[bool, str]:
        goal = NavigateToPoint.Goal()
        goal.target_x = self.target_x
        goal.target_y = self.target_y
        goal.linear_speed = self.linear_speed
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.navigation_timeout_sec
        if not self._navigation_client.wait_for_server(
            timeout_sec=self.action_wait_timeout_sec
        ):
            return False, f"Action server unavailable: {self.navigation_action}"
        return self._send_goal_and_wait(
            self._navigation_client,
            goal,
            self._handle_navigation_feedback,
        )

    def _run_spin_step(self) -> Tuple[bool, str]:
        goal = Spin.Goal()
        goal.rotations = self.rotations
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.spin_timeout_sec
        if not self._spin_client.wait_for_server(timeout_sec=self.action_wait_timeout_sec):
            return False, f"Action server unavailable: {self.spin_action}"
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
        result_holder = {"success": False, "message": "Action did not finish"}

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
            goal_handle.get_result_async().add_done_callback(handle_result)

        def handle_result(future) -> None:
            try:
                result = future.result().result
                result_holder["success"] = bool(result.success)
                result_holder["message"] = str(result.message)
            except Exception as exception:
                result_holder["message"] = f"Action result failed: {exception}"
            with self._lock:
                if self._active_goal_handle is goal_handle_holder["handle"]:
                    self._active_goal_handle = None
            done_event.set()

        try:
            client.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            ).add_done_callback(handle_goal_response)
        except Exception as exception:
            return False, f"Failed to send action goal: {exception}"

        start_time = monotonic()
        while not done_event.wait(timeout=0.05):
            if self._is_stop_requested():
                abandoned_event.set()
                self._cancel_active_goal()
                return False, "Mission stop requested"
            if timeout_sec is not None and monotonic() - start_time >= timeout_sec:
                abandoned_event.set()
                self._cancel_active_goal()
                return False, f"Action timed out after {timeout_sec:.2f} seconds"
        return bool(result_holder["success"]), str(result_holder["message"])

    def _cancel_active_goal(self) -> None:
        with self._lock:
            active_goal_handle = self._active_goal_handle
            self._active_goal_handle = None
        if active_goal_handle is not None:
            active_goal_handle.cancel_goal_async()

    def _handle_navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Step1 feedback: {feedback.state}, "
            f"distance={feedback.distance_remaining:.2f}"
        )

    def _handle_safe_navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Step1 feedback: {feedback.state}, "
            f"distance={feedback.distance_remaining:.2f}"
        )

    def _handle_spin_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Step2 feedback: {feedback.state}, "
            f"rotation_remaining={feedback.rotation_remaining:.2f}"
        )

    def _handle_arm_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(f"Arm action progress={feedback.progress:.2f}")

    def _handle_gripper_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(f"Gripper current state={feedback.current_state}")

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
        return None if pose_age > self.pose_timeout_sec else pose

    def _get_fresh_cushion_pose(self) -> Optional[Tuple[float, float, float]]:
        with self._cushion_pose_lock:
            pose = self._latest_cushion_pose
            pose_time = self._latest_cushion_pose_time
        if pose is None or pose_time is None:
            return None
        pose_age = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        return None if pose_age > self.pose_timeout_sec else pose

    def _is_stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    def _stop_robot(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _publish_status(self, status: str) -> None:
        self._status_publisher.publish(String(data=status))


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
