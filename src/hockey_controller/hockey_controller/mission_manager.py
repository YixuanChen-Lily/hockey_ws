#!/usr/bin/env python3

import math
from threading import Event, Thread
from time import monotonic
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
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
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy


from hockey_interfaces.action import NavigateToPoint
from hockey_interfaces.action import ShootPuck
from hockey_interfaces.action import Spin
from robomaster_msgs.action import GripperControl
from robomaster_msgs.action import MoveArm

from hockey_controller.cushion_parking_planner import CushionGeometry
from hockey_controller.cushion_parking_planner import ParkingPlannerConfig
from hockey_controller.cushion_parking_planner import plan_parking_route
from hockey_controller.control_utils import wrap_to_pi
from hockey_controller.control_utils import yaw_from_quaternion
from hockey_controller.parking_markers import marker_array_type
from hockey_controller.parking_markers import publish_parking_markers
from hockey_controller.parking_markers import visualization_available


class MissionManager(Node):
    """Small state-machine wrapper for cushion parking."""

    DEFAULTS = {
        "safe_navigation_action": "safe_navigate_to_point",
        "spin_action": "spin",
        "shooting_action": "shoot_puck",
        "robot_id": 1,
        "pose_topic": "",
        "cushion_pose_topic": "",
        "goal_pose_topic": "/vrpn_mocap/goal/pose",
        "parking_enabled": True,
        "target_x": 1.0,
        "target_y": 0.0,
        "cushion_length": 1.0,
        "cushion_width": 0.12,
        "parking_front_axis": "y",
        "front_normal_sign": -1.0,
        "front_side_threshold": 0.0,
        "side_clearance": 0.35,
        "front_clearance": 0.35,
        "desired_normal_distance": 0.35,
        "parking_lateral_offset": 0.0,
        "pre_park_backoff": 0.40,
        "parking_robot_safety_radius": 0.20,
        "parking_safety_margin": 0.10,
        "cushion_circle_spacing": 0.20,
        "cushion_obstacle_axis": "local_x",
        "cushion_obstacle_radius_override": -1.0,
        "parking_lookahead_distance": 0.25,
        "final_approach_speed": 0.12,
        "align_gain": 2.0,
        "align_timeout_sec": 150.0,
        "final_yaw_tolerance": 0.08,
        "pose_timeout_sec": 150.0,
        "visualization_frame": "map",
        "rotations": 1,
        "linear_speed": 0.4,
        "angular_speed": 0.8,
        "safe_navigation_timeout_sec": 150.0,
        "spin_timeout_sec": 150.0,
        "action_wait_timeout_sec": 150.0,
        "shooting_enabled": False,
        "shooting_role": "single",  # shooter, passer, or single
        "team_name": "team_rocket",
        "teammate_robot_id": 0,
        "team_wait_timeout_sec": 150.0,
        "shooting_offset_x": 0.0,
        "shooting_offset_y": 0.0,
        "shooting_target_radius": 0.20,
        "shooting_approach_distance": 0.05,
        "shooting_contact_gap": 0.0,
        "shooting_spin_direction": "ccw",
        "shooting_angle_offset": 0.0,
        "shooting_linear_speed": 0.3,
        "shooting_angular_speed": 3.0,
        "shooting_spin_rotations": 1,
        "shooting_spin_angle_deg": 30.0,
        "shooting_timeout_sec": 150.0,
        "shooting_max_attempts": 20,
        "use_manipulator": True,
        "reset_arm_x": 0.0,
        "reset_arm_z": 0.0,
        "reset_arm_settle_sec": 0.5,
        "grab_arm_x": 0.3,
        "grab_arm_z": 0.3,
        "grab_arm_relative": False,
        "grab_arm_settle_sec": 0.5,
        "gripper_open_power": 0.5,
        "gripper_open_settle_sec": 0.5,
        "gripper_close_power": 0.5,
        "gripper_close_settle_sec": 0.5,
        "lift_arm_x": 0.0,
        "lift_arm_z": 1.0,
        "lift_arm_relative": False,
        "lift_arm_settle_sec": 0.5,
        "backward_distance": 0.30,
        "backward_duration_sec": 2.0,
        "backward_publish_rate_hz": 20.0,
        "ready_arm_x": 0.1,
        "ready_arm_z": 0.0,
        "ready_arm_relative": False,
        "ready_arm_settle_sec": 0.5,
    }

    def __init__(self) -> None:
        super().__init__("mission_manager")
        for name, default in self.DEFAULTS.items():
            self.declare_parameter(name, default)

        self._group = ReentrantCallbackGroup()
        self._latest_pose = None
        self._latest_pose_time = None
        self._latest_cushion_pose = None
        self._latest_cushion_pose_time = None
        self._latest_goal_pose = None
        self._latest_goal_pose_time = None
        self._running = False
        self._stop_requested = False
        self._active_goal = None
        self._team_pass_done = Event()
        self._latest_team_status = ""

        self._load_parameters()
        self.pose_topic = self.pose_topic or f"/vrpn_mocap/dji_robot_{self.robot_id}/pose"
        self.cushion_pose_topic = self.cushion_pose_topic or "/vrpn_mocap/hockey_sticks_1/pose"
        self.cmd_vel_topic = f"/robot{self.robot_id}/cmd_vel"
        self.arm_action = f"/robot{self.robot_id}/move_arm"
        self.gripper_action = f"/robot{self.robot_id}/gripper"

        self.status_pub = self.create_publisher(String, "mission/status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.marker_pub = None
        if visualization_available():
            self.marker_pub = self.create_publisher(
                marker_array_type(),
                "mission/parking_markers",
                10,
            )

        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.cushion_sub = self.create_subscription(
            PoseStamped,
            self.cushion_pose_topic,
            self._cushion_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self._goal_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_service(
            Trigger,
            "mission/start",
            self._start_service,
            callback_group=self._group,
        )
        self.create_service(
            Trigger,
            "mission/stop",
            self._stop_service,
            callback_group=self._group,
        )
        self.add_on_set_parameters_callback(self._set_parameters_callback)

        self.safe_nav_client = ActionClient(
            self,
            NavigateToPoint,
            self.safe_navigation_action,
            callback_group=self._group,
        )
        self.spin_client = ActionClient(
            self,
            Spin,
            self.spin_action,
            callback_group=self._group,
        )
        self.shooting_client = ActionClient(
            self,
            ShootPuck,
            self.shooting_action,
            callback_group=self._group,
        )
        self.arm_client = ActionClient(
            self,
            MoveArm,
            self.arm_action,
            callback_group=self._group,
        )
        self.gripper_client = ActionClient(
            self,
            GripperControl,
            self.gripper_action,
            callback_group=self._group,
        )
        self.safe_param_client = self.create_client(
            SetParameters,
            "safe_navigation_server/set_parameters",
            callback_group=self._group,
        )


        self.team_status_topic = (
            f"/{self.team_name}/robot_{self.robot_id}/passer/status"
        )
        teammate_id = self.teammate_robot_id or self.robot_id
        self.team_listen_topic = (
            f"/{self.team_name}/robot_{teammate_id}/passer/status"
        )

        team_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.team_publisher = self.create_publisher(
            String,
            self.team_status_topic,
            team_qos,
        )
        self.team_subscription = self.create_subscription(
            String,
            self.team_listen_topic,
            self.team_status_callback,
            team_qos,
        )

        self._status("IDLE")
        self.get_logger().info(
            "Mission manager ready. Call /mission/start.\n"
            f"  pose={self.pose_topic}\n"
            f"  cushion={self.cushion_pose_topic}\n"
            f"  robot_id={self.robot_id}\n"
            f"  parking={self.parking_enabled}\n"
            f"  shooting={self.shooting_enabled} ({self.shooting_role})\n"
            f"  team pub={self.team_status_topic}\n"
            f"  team listen={self.team_listen_topic}"
        )

    def team_status_callback(self, msg):
        status = str(msg.data)
        self._latest_team_status = status
        self.get_logger().info(f"Received team status: {status}")
        if status in ("PASS_DONE", "PASSER_DONE", "PUCK_PASSED"):
            self._team_pass_done.set()

    def _load_parameters(self) -> None:
        for name, default in self.DEFAULTS.items():
            value = self.get_parameter(name).value
            if isinstance(default, bool):
                value = bool(value)
            elif isinstance(default, int):
                value = int(value)
            elif isinstance(default, float):
                value = float(value)
            elif isinstance(default, str):
                value = str(value)
            setattr(self, name, value)

    def _set_parameters_callback(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "cushion_pose_topic":
                self._reset_cushion_topic(
                    str(parameter.value) or "/vrpn_mocap/hockey_sticks_1/pose"
                )
        return SetParametersResult(successful=True)

    def _reset_cushion_topic(self, topic: str) -> None:
        if topic == self.cushion_pose_topic:
            return
        self.destroy_subscription(self.cushion_sub)
        self.cushion_pose_topic = topic
        self._latest_cushion_pose = None
        self._latest_cushion_pose_time = None
        self.cushion_sub = self.create_subscription(
            PoseStamped,
            topic,
            self._cushion_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.get_logger().info(f"Updated cushion pose topic: {topic}")

    def _start_service(self, request, response):
        del request
        if self._running:
            response.success = False
            response.message = "Mission already running"
            return response
        self._load_parameters()
        self._running = True
        self._stop_requested = False
        Thread(target=self._run_mission, daemon=True).start()
        response.success = True
        response.message = "Mission started"
        return response

    def _run_mission(self) -> None:
        try:
            if self.use_manipulator:
                self._reset_arm()
                self._reset_gripper()
            if self.parking_enabled:
                self._run_parking_mission()
            else:
                self._run_simple_mission()
        except Exception as exception:
            self._status("MISSION_FAILED")
            self.get_logger().error(f"Mission failed: {exception}")
        finally:
            self._stop_robot()
            self._active_goal = None
            self._running = False
            self._stop_requested = False

    def _stop_service(self, request, response):
        del request
        self._stop_requested = True
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()
        self._stop_robot()
        self._status("MISSION_STOPPED")
        response.success = True
        response.message = "Stop requested"
        return response

    def _run_parking_mission(self) -> None:
        self._status("PARKING")
        robot_pose = self._fresh_robot_pose()
        cushion_pose = self._fresh_cushion_pose()
        if robot_pose is None or cushion_pose is None:
            raise RuntimeError("stale robot or cushion pose")

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
            parking_lateral_offset=self.parking_lateral_offset,
            pre_park_backoff=self.pre_park_backoff,
            robot_safety_radius=self.parking_robot_safety_radius,
            safety_margin=self.parking_safety_margin,
            circle_spacing=self.cushion_circle_spacing,
            obstacle_axis=self.cushion_obstacle_axis,
            obstacle_radius_override=self.cushion_obstacle_radius_override,
        )
        plan = plan_parking_route((robot_pose[0], robot_pose[1]), geometry, config)
        if not plan.waypoints:
            raise RuntimeError(plan.message)

        self._publish_parking_markers(robot_pose, geometry, config, plan)
        self._configure_safe_nav_obstacles(plan)

        if plan.already_front_side:
            self.get_logger().info("Robot already on cushion parking side")
        else:
            self.get_logger().info(f"Selected {plan.bypass_side} bypass route")
            for waypoint in plan.waypoints[:-2]:
                self._status("GO_TO_BYPASS_WAYPOINT")
                self._safe_navigate(waypoint, self.linear_speed)

        self._status("GO_TO_PRE_PARK_POINT")
        self._safe_navigate(plan.pre_park_point, self.linear_speed)

        self._status("ALIGN_FOR_FINAL_APPROACH")
        self._align_to_yaw(plan.final_yaw)

        self._status("FINAL_APPROACH")
        final_goal = self._control_point_goal(plan.final_park_point, plan.final_yaw)
        self._safe_navigate(final_goal, self.final_approach_speed)
        self._align_to_yaw(plan.final_yaw)

        self._status("PARKING_DONE")
        if self.use_manipulator:
            self._pick_up_stick()
        else:
            self.get_logger().info("Skipping stick pickup because use_manipulator=false")
        if self.shooting_enabled:
            if self.shooting_role == "passer":
                self.get_logger().info("Shooting to pass location because role=passer")
                self._shoot_puck()
                self._retreat_to_cushion_side(plan)
                self._publish_team_status("PASS_DONE")

            elif self.shooting_role == "shooter":
                self.get_logger().info("Waiting to shoot because role=shooter")
                self._go_to_shooter_wait_pose()
                self._wait_for_team_pass()
                self._shoot_puck()

            elif self.shooting_role == "single":
                self.get_logger().info("Shooting puck because role=single")
                self._shoot_puck()
                self._retreat_to_cushion_side(plan)
            else:
                raise RuntimeError(
                    f"unsupported shooting_role {self.shooting_role!r}; "
                    "use single, passer, or shooter"
                )
        self._status("MISSION_DONE")

    def _run_simple_mission(self) -> None:
        self._status("NAVIGATE")
        self._safe_navigate((self.target_x, self.target_y), self.linear_speed)
        self._status("SPIN")
        self._spin()
        self._status("MISSION_DONE")

    def _publish_parking_markers(self, robot_pose, geometry, config, plan) -> None:
        self.get_logger().info(
            "Parking obstacles: "
            f"{[(round(o.x, 3), round(o.y, 3), round(o.radius, 3)) for o in plan.cushion_obstacles]}"
        )
        publish_parking_markers(
            self,
            self.marker_pub,
            self.visualization_frame,
            robot_pose,
            geometry,
            config,
            plan,
        )

    def _configure_safe_nav_obstacles(self, plan) -> None:
        self._set_safe_nav_parameters(
            {
                "use_target_pose": False,
                "orient_to_target": False,
                "robot_safety_radius": 0.0,
                "obstacle_safe_margin": 0.0,
                "obstacle_x": [obstacle.x for obstacle in plan.cushion_obstacles],
                "obstacle_y": [obstacle.y for obstacle in plan.cushion_obstacles],
                "obstacle_radius": [
                    obstacle.radius for obstacle in plan.cushion_obstacles
                ],
            }
        )

    def _safe_navigate(self, point, speed: float) -> None:
        goal = NavigateToPoint.Goal()
        goal.target_x = float(point[0])
        goal.target_y = float(point[1])
        goal.linear_speed = float(speed)
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.safe_navigation_timeout_sec
        self._send_goal(self.safe_nav_client, goal, self._navigation_feedback)

    def _spin(self) -> None:
        goal = Spin.Goal()
        goal.rotations = self.rotations
        goal.spin_angle_deg = 0.0
        goal.angular_speed = self.angular_speed
        goal.timeout_sec = self.spin_timeout_sec
        self._send_goal(self.spin_client, goal, self._spin_feedback)

    def _shoot_puck(self) -> None:
        self._status("SHOOTING")
        goal = ShootPuck.Goal()
        goal.role = self.shooting_role
        goal.offset_x = self.shooting_offset_x
        goal.offset_y = self.shooting_offset_y
        goal.target_radius = self.shooting_target_radius
        goal.approach_distance = self.shooting_approach_distance
        goal.contact_gap = self.shooting_contact_gap
        goal.spin_direction = self.shooting_spin_direction
        goal.shooting_angle_offset = self.shooting_angle_offset
        goal.linear_speed = self.shooting_linear_speed
        goal.angular_speed = self.shooting_angular_speed
        goal.spin_rotations = self.shooting_spin_rotations
        goal.spin_angle_deg = self.shooting_spin_angle_deg
        goal.timeout_sec = self.shooting_timeout_sec
        goal.max_attempts = self.shooting_max_attempts
        self._send_goal(self.shooting_client, goal, self._shooting_feedback)

    def _retreat_to_cushion_side(self, plan) -> None:
        self._status("RETREAT_TO_CUSHION_SIDE")
        self._safe_navigate(plan.pre_park_point, self.linear_speed)
        self._align_to_yaw(plan.final_yaw)

    def _go_to_shooter_wait_pose(self) -> None:
        goal_pose = self._fresh_goal_pose()
        if goal_pose is None:
            raise RuntimeError("stale goal pose before shooter wait")
        wait_point = self._shooter_wait_point(goal_pose)
        self._status("GO_TO_SHOOTER_WAIT_POSE")
        self._safe_navigate(wait_point, self.linear_speed)

    def _shooter_wait_point(
        self,
        goal_pose: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        cos_goal = math.cos(goal_pose[2])
        sin_goal = math.sin(goal_pose[2])
        target_x = (
            goal_pose[0]
            + self.shooting_offset_x * cos_goal
            - self.shooting_offset_y * sin_goal
        )
        target_y = (
            goal_pose[1]
            + self.shooting_offset_x * sin_goal
            + self.shooting_offset_y * cos_goal
        )
        return target_x, target_y

    def _publish_team_status(self, status: str) -> None:
        self.team_publisher.publish(String(data=status))
        self.get_logger().info(f"Published team status: {status}")

    def _wait_for_team_pass(self) -> None:
        self._status("WAIT_FOR_PASS")
        if self._team_pass_done.is_set():
            return

        self.get_logger().info(
            "Waiting for passer status "
            f"on {self.team_listen_topic} for up to "
            f"{self.team_wait_timeout_sec:.1f} seconds"
        )
        if not self._team_pass_done.wait(timeout=self.team_wait_timeout_sec):
            raise RuntimeError(
                "timed out waiting for PASS_DONE from passer; "
                f"last team status={self._latest_team_status!r}"
            )

    def _send_goal(self, client, goal, feedback_callback) -> None:
        if not client.wait_for_server(timeout_sec=self.action_wait_timeout_sec):
            raise RuntimeError("action server unavailable")

        done = Event()
        outcome = {"success": False, "message": "No result"}

        def goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                outcome["message"] = "Goal rejected"
                done.set()
                return
            self._active_goal = goal_handle
            goal_handle.get_result_async().add_done_callback(goal_result)

        def goal_result(future):
            wrapped_result = future.result()
            result = wrapped_result.result
            if hasattr(result, "success"):
                outcome["success"] = bool(result.success)
                outcome["message"] = str(result.message)
            else:
                outcome["success"] = (
                    wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                )
                outcome["message"] = (
                    "Action succeeded"
                    if outcome["success"]
                    else f"Action failed with status {wrapped_result.status}"
                )
            self._active_goal = None
            done.set()

        client.send_goal_async(goal, feedback_callback=feedback_callback).add_done_callback(
            goal_response
        )
        while rclpy.ok() and not done.wait(0.05):
            if self._stop_requested:
                raise RuntimeError("Mission stopped")
        if not outcome["success"]:
            raise RuntimeError(outcome["message"])

    def _align_to_yaw(self, target_yaw: float) -> None:
        pose = self._fresh_robot_pose()
        if pose is None:
            raise RuntimeError("stale robot pose during align")
        error = wrap_to_pi(target_yaw - pose[2])
        self.get_logger().info(
            "parking align check: "
            f"target_yaw={target_yaw:.3f}, "
            f"current_yaw={pose[2]:.3f}, "
            f"error={error:.3f}, "
            f"tolerance={self.final_yaw_tolerance:.3f}"
        )
        if abs(error) <= self.final_yaw_tolerance:
            self._stop_robot()
            self.get_logger().info("parking align skipped: already within tolerance")
            return

        goal = Spin.Goal()
        goal.rotations = 0
        goal.spin_angle_deg = math.degrees(abs(error))
        goal.angular_speed = math.copysign(abs(self.angular_speed), error)
        goal.timeout_sec = self.align_timeout_sec
        self.get_logger().info(
            "parking align spin goal: "
            f"spin_angle_deg={goal.spin_angle_deg:.1f}, "
            f"angular_speed={goal.angular_speed:.3f}, "
            f"timeout={goal.timeout_sec:.1f}s"
        )
        self._send_goal(self.spin_client, goal, self._spin_feedback)

    def _pick_up_stick(self) -> None:
        self._status("PICK_UP_STICK")
        self._open_gripper()
        self._move_arm(
            self.grab_arm_x,
            self.grab_arm_z,
            self.grab_arm_relative,
            self.grab_arm_settle_sec,
        )
        self._close_gripper()
        self._back_up()

    def _reset_arm(self) -> None:
        """Move the arm to its configured absolute home pose."""
        self._status("RESET_ARM")
        self.get_logger().info(
            "Resetting arm to absolute pose: "
            f"x={float(self.reset_arm_x):.3f}m, "
            f"z={float(self.reset_arm_z):.3f}m."
        )
        self._move_arm(
            self.reset_arm_x,
            self.reset_arm_z,
            False,
            self.reset_arm_settle_sec,
        )

    def _reset_gripper(self) -> None:
        """Reset the gripper to its open state."""
        self._status("RESET_GRIPPER")
        self.get_logger().info("Resetting gripper to the open state.")
        self._open_gripper()

    def _move_arm(
        self,
        x: float,
        z: float,
        relative: bool,
        settle_sec: float,
    ) -> None:
        goal = MoveArm.Goal()
        goal.x = float(x)
        goal.z = float(z)
        goal.relative = bool(relative)
        self._send_goal(self.arm_client, goal, self._arm_feedback)
        Event().wait(settle_sec)

    def _open_gripper(self) -> None:
        goal = GripperControl.Goal()
        goal.target_state = GripperControl.Goal.OPEN
        goal.power = self.gripper_open_power
        self._send_goal(self.gripper_client, goal, self._gripper_feedback)
        Event().wait(self.gripper_open_settle_sec)

    def _close_gripper(self) -> None:
        goal = GripperControl.Goal()
        goal.target_state = GripperControl.Goal.CLOSE
        goal.power = self.gripper_close_power
        self._send_goal(self.gripper_client, goal, self._gripper_feedback)
        Event().wait(self.gripper_close_settle_sec)

    def _back_up(self) -> None:
        duration = float(self.backward_duration_sec)
        publish_rate_hz = float(self.backward_publish_rate_hz)
        if duration <= 0.0:
            raise RuntimeError("backward_duration_sec must be positive")
        if publish_rate_hz <= 0.0:
            raise RuntimeError("backward_publish_rate_hz must be positive")

        twist = Twist()
        twist.linear.x = -float(self.backward_distance) / duration
        self.get_logger().info(
            "Backing up: "
            f"distance={float(self.backward_distance):.3f}m, "
            f"duration={duration:.2f}s, velocity={twist.linear.x:.3f}m/s."
        )
        start = monotonic()
        while monotonic() - start < duration:
            if self._stop_requested:
                raise RuntimeError("Mission stopped during backup")
            self.cmd_vel_pub.publish(twist)
            Event().wait(1.0 / publish_rate_hz)
        self._stop_robot()
        self.get_logger().info("Backing up completed.")

    def _set_safe_nav_parameters(self, values) -> None:
        if not self.safe_param_client.wait_for_service(
            timeout_sec=self.action_wait_timeout_sec
        ):
            raise RuntimeError("safe_navigation_server parameter service unavailable")
        request = SetParameters.Request()
        request.parameters = [
            self._parameter_message(name, value)
            for name, value in values.items()
        ]
        future = self.safe_param_client.call_async(request)
        while rclpy.ok() and not future.done():
            Event().wait(0.05)
        for result in future.result().results:
            if not result.successful:
                raise RuntimeError(f"failed to set safe nav parameter: {result.reason}")

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

    def _control_point_goal(self, position, yaw: float) -> Tuple[float, float]:
        return (
            position[0] + self.parking_lookahead_distance * math.cos(yaw),
            position[1] + self.parking_lookahead_distance * math.sin(yaw),
        )

    def _fresh_robot_pose(self):
        if self._latest_pose is None or self._latest_pose_time is None:
            return None
        age = (self.get_clock().now() - self._latest_pose_time).nanoseconds / 1e9
        return self._latest_pose if age <= self.pose_timeout_sec else None

    def _fresh_cushion_pose(self):
        if self._latest_cushion_pose is None or self._latest_cushion_pose_time is None:
            return None
        age = (
            self.get_clock().now() - self._latest_cushion_pose_time
        ).nanoseconds / 1e9
        return self._latest_cushion_pose if age <= self.pose_timeout_sec else None

    def _fresh_goal_pose(self):
        if self._latest_goal_pose is None or self._latest_goal_pose_time is None:
            return None
        age = (
            self.get_clock().now() - self._latest_goal_pose_time
        ).nanoseconds / 1e9
        return self._latest_goal_pose if age <= self.pose_timeout_sec else None

    def _pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        self._latest_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self._latest_pose_time = self.get_clock().now()

    def _cushion_pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        self._latest_cushion_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self._latest_cushion_pose_time = self.get_clock().now()

    def _goal_pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        self._latest_goal_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self._latest_goal_pose_time = self.get_clock().now()

    def _navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"navigation: {feedback.state}, distance={feedback.distance_remaining:.2f}"
        )

    def _spin_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"spin: {feedback.state}, remaining={feedback.rotation_remaining:.2f}"
        )

    def _shooting_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            "shooting: "
            f"{feedback.state}, distance={feedback.puck_distance_to_target:.2f}, "
            f"attempts={feedback.attempts}"
        )

    def _arm_feedback(self, feedback_message) -> None:
        self.get_logger().info(f"arm progress={feedback_message.feedback.progress:.2f}")

    def _gripper_feedback(self, feedback_message) -> None:
        self.get_logger().info(
            f"gripper state={feedback_message.feedback.current_state}"
        )

    def _stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))


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
