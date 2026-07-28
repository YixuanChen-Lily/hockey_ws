#!/usr/bin/env python3
import math
from threading import Event
from time import monotonic
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from hockey_interfaces.action import GripperControl, MoveArm, NavigateToPoint
from hockey_controller.cushion_parking_planner import CushionGeometry, ParkingPlannerConfig, plan_parking_route
from hockey_controller.navigation_server import clamp, wrap_to_pi, yaw_from_quaternion


class MissionManager(Node):
    """Run the parking-and-stick-pickup course mission once."""

    def __init__(self) -> None:
        super().__init__("mission_manager")
        defaults = {
            "safe_navigation_action": "safe_navigate_to_point",
            "arm_action": "control_arm", "gripper_action": "control_gripper",
            "robot_id": 1, "pose_topic": "", "cushion_pose_topic": "",
            "cushion_length": 1.0, "cushion_width": 0.12,
            "parking_front_axis": "y", "front_normal_sign": -1.0,
            "front_side_threshold": 0.0, "side_clearance": 0.35,
            "front_clearance": 0.35, "desired_normal_distance": 0.35,
            "tangential_offset": 0.0,
            "pre_park_backoff": 0.40, "parking_robot_safety_radius": 0.20,
            "stick_safety_extension": 0.0, "parking_safety_margin": 0.10,
            "cushion_circle_spacing": 0.20,
            "parking_lookahead_distance": 0.25, "linear_speed": 0.4,
            "angular_speed": 0.8, "final_approach_speed": 0.12,
            "final_approach_point_gain": 0.35, "align_gain": 2.0,
            "final_yaw_tolerance": 0.08,
            "grab_arm_x": 0.0, "grab_arm_z": 0.0,
            "grab_arm_relative": False, "grab_arm_settle_sec": 0.3,
            "gripper_close_power": 0.5, "gripper_close_settle_sec": 0.5,
            "lift_arm_x": 1.0, "lift_arm_z": 2.0,
            "lift_arm_relative": False, "lift_arm_settle_sec": 0.3,
            "backward_distance": 0.30, "backward_duration_sec": 2.0,
            "backward_publish_rate_hz": 20.0,
            "ready_arm_x": 0.0, "ready_arm_z": 0.0,
            "ready_arm_relative": False, "ready_arm_settle_sec": 0.3,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        self.robot_id = int(self.robot_id)
        self.pose_topic = str(self.pose_topic) or f"/vrpn_mocap/dji_robot_{self.robot_id}/pose"
        self.cushion_pose_topic = str(self.cushion_pose_topic) or "/vrpn_mocap/hockey_sticks_1/pose"
        self.cmd_vel_topic = f"/robot{self.robot_id}/cmd_vel"
        self._latest_pose = None
        self._latest_cushion_pose = None
        group = ReentrantCallbackGroup()
        self._status_publisher = self.create_publisher(String, "mission/status", 10)
        self._cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_callback,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(PoseStamped, self.cushion_pose_topic,
                                 self._cushion_pose_callback, qos_profile_sensor_data,
                                 callback_group=group)
        self._safe_navigation_client = ActionClient(
            self, NavigateToPoint, str(self.safe_navigation_action), callback_group=group)
        self._arm_client = ActionClient(self, MoveArm, str(self.arm_action), callback_group=group)
        self._gripper_client = ActionClient(
            self, GripperControl, str(self.gripper_action), callback_group=group)
        self._safe_nav_parameter_client = self.create_client(
            SetParameters, "safe_navigation_server/set_parameters", callback_group=group)
        self._start_timer = self.create_timer(0.1, self._start_when_pose_is_ready,
                                              callback_group=group)
        self._publish_status("WAITING_POSE")
        self.get_logger().info("Waiting for robot and cushion poses.")

    def _start_when_pose_is_ready(self) -> None:
        if self._latest_pose is None or self._latest_cushion_pose is None:
            return
        self._start_timer.cancel()
        self._run_parking_mission()

    def _run_parking_mission(self) -> None:
        self._publish_status("PARKING")
        self.get_logger().info("Parking started.")
        robot_pose, cushion_pose = self._latest_pose, self._latest_cushion_pose
        geometry = CushionGeometry(
            center_x=cushion_pose[0], center_y=cushion_pose[1],
            length=float(self.cushion_length), width=float(self.cushion_width),
            yaw=cushion_pose[2], front_axis=str(self.parking_front_axis),
            front_normal_sign=float(self.front_normal_sign))
        config = ParkingPlannerConfig(
            front_side_threshold=float(self.front_side_threshold),
            side_clearance=float(self.side_clearance), front_clearance=float(self.front_clearance),
            desired_normal_distance=float(self.desired_normal_distance),
            tangential_offset=float(self.tangential_offset),
            pre_park_backoff=float(self.pre_park_backoff),
            robot_safety_radius=float(self.parking_robot_safety_radius),
            stick_safety_extension=float(self.stick_safety_extension),
            safety_margin=float(self.parking_safety_margin),
            circle_spacing=float(self.cushion_circle_spacing))
        plan = plan_parking_route((robot_pose[0], robot_pose[1]), geometry, config)
        self._configure_safe_navigation(plan)
        for waypoint in plan.waypoints[:-2]:
            self._navigate(waypoint, self.linear_speed)
        self._navigate(plan.pre_park_point, self.linear_speed)
        self._align_to_yaw(plan.final_yaw)
        self._set_safe_nav_parameters({"point_gain": float(self.final_approach_point_gain)})
        final_point = self._control_point_goal(plan.final_park_point, plan.final_yaw)
        self._navigate(final_point, self.final_approach_speed)
        self._align_to_yaw(plan.final_yaw)

        self._publish_status("PICKING_STICK")
        self.get_logger().info("Parking complete; picking up hockey stick.")
        self._run_stick_setup_mission()
        self._publish_status("DONE")
        self.get_logger().info("Parking-and-pickup mission completed.")

    def _run_stick_setup_mission(self) -> None:
        self._move_arm(self.grab_arm_x, self.grab_arm_z,
                       self.grab_arm_relative, self.grab_arm_settle_sec)
        self._close_gripper()
        self._move_arm(self.lift_arm_x, self.lift_arm_z,
                       self.lift_arm_relative, self.lift_arm_settle_sec)
        self._back_up()
        self._move_arm(self.ready_arm_x, self.ready_arm_z,
                       self.ready_arm_relative, self.ready_arm_settle_sec)

    def _move_arm(self, x, z, relative, settle_sec) -> None:
        goal = MoveArm.Goal()
        goal.x, goal.z, goal.relative = float(x), float(z), bool(relative)
        self._send_goal_and_wait(self._arm_client, goal)
        Event().wait(float(settle_sec))

    def _close_gripper(self) -> None:
        goal = GripperControl.Goal()
        goal.target_state = GripperControl.Goal.CLOSE
        goal.power = float(self.gripper_close_power)
        self._send_goal_and_wait(self._gripper_client, goal)
        Event().wait(float(self.gripper_close_settle_sec))

    def _back_up(self) -> None:
        duration = float(self.backward_duration_sec)
        twist = Twist()
        twist.linear.x = -float(self.backward_distance) / duration
        start = monotonic()
        while monotonic() - start < duration:
            self._cmd_vel_publisher.publish(twist)
            Event().wait(1.0 / float(self.backward_publish_rate_hz))
        self._stop_robot()

    def _navigate(self, point, speed) -> None:
        goal = NavigateToPoint.Goal()
        goal.target_x, goal.target_y = float(point[0]), float(point[1])
        goal.linear_speed, goal.angular_speed = float(speed), float(self.angular_speed)
        goal.timeout_sec = 30.0
        self._send_goal_and_wait(self._safe_navigation_client, goal)

    def _align_to_yaw(self, target_yaw) -> None:
        while True:
            error = wrap_to_pi(target_yaw - self._latest_pose[2])
            if abs(error) <= float(self.final_yaw_tolerance):
                self._stop_robot()
                return
            twist = Twist()
            twist.angular.z = clamp(float(self.align_gain) * error,
                                    -float(self.angular_speed), float(self.angular_speed))
            self._cmd_vel_publisher.publish(twist)
            Event().wait(0.05)

    def _configure_safe_navigation(self, plan) -> None:
        self._set_safe_nav_parameters({
            "use_target_pose": False, "orient_to_target": False,
            "obstacles_enabled": True, "robot_safety_radius": 0.0,
            "obstacle_safe_margin": 0.0,
            "obstacle_x": [o.x for o in plan.cushion_obstacles],
            "obstacle_y": [o.y for o in plan.cushion_obstacles],
            "obstacle_radius": [o.radius for o in plan.cushion_obstacles]})

    def _set_safe_nav_parameters(self, values) -> None:
        self._safe_nav_parameter_client.wait_for_service()
        request = SetParameters.Request()
        request.parameters = [self._parameter_message(k, v) for k, v in values.items()]
        future = self._safe_nav_parameter_client.call_async(request)
        while not future.done():
            Event().wait(0.05)

    @staticmethod
    def _parameter_message(name, value):
        parameter = Parameter(name=name, value=ParameterValue())
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter.value.double_array_value = [float(item) for item in value]
        return parameter

    def _control_point_goal(self, position, yaw):
        distance = float(self.parking_lookahead_distance)
        return (position[0] + distance * math.cos(yaw),
                position[1] + distance * math.sin(yaw))

    @staticmethod
    def _send_goal_and_wait(client, goal) -> None:
        client.wait_for_server()
        done = Event()

        def accepted(future):
            result_future = future.result().get_result_async()
            result_future.add_done_callback(lambda _: done.set())

        client.send_goal_async(goal).add_done_callback(accepted)
        done.wait()

    def _pose_callback(self, message) -> None:
        pose = message.pose
        self._latest_pose = (float(pose.position.x), float(pose.position.y),
                             yaw_from_quaternion(pose.orientation))

    def _cushion_pose_callback(self, message) -> None:
        pose = message.pose
        self._latest_cushion_pose = (float(pose.position.x), float(pose.position.y),
                                     yaw_from_quaternion(pose.orientation))

    def _stop_robot(self) -> None:
        self._cmd_vel_publisher.publish(Twist())

    def _publish_status(self, status) -> None:
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
