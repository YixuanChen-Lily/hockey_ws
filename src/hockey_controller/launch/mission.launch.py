from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    robot_id = LaunchConfiguration("robot_id")
    target_pose_topic = LaunchConfiguration("target_pose_topic")

    safe_qp_solver = LaunchConfiguration("safe_qp_solver")
    safe_dynamic_robot_ids = LaunchConfiguration("safe_dynamic_robot_ids")
    safe_dynamic_obstacles_required = LaunchConfiguration(
        "safe_dynamic_obstacles_required"
    )
    safe_dynamic_controlled_robot_radius = LaunchConfiguration(
        "safe_dynamic_controlled_robot_radius"
    )
    safe_dynamic_robot_radius = LaunchConfiguration("safe_dynamic_robot_radius")
    safe_dynamic_robot_safety_margin = LaunchConfiguration(
        "safe_dynamic_robot_safety_margin"
    )

    parking_enabled = LaunchConfiguration("parking_enabled")
    cushion_length = LaunchConfiguration("cushion_length")
    cushion_width = LaunchConfiguration("cushion_width")
    parking_front_axis = LaunchConfiguration("parking_front_axis")
    front_normal_sign = LaunchConfiguration("front_normal_sign")
    cushion_obstacle_axis = LaunchConfiguration("cushion_obstacle_axis")
    desired_normal_distance = LaunchConfiguration("desired_normal_distance")
    parking_lateral_offset = LaunchConfiguration("parking_lateral_offset")
    pre_park_backoff = LaunchConfiguration("pre_park_backoff")
    side_clearance = LaunchConfiguration("side_clearance")
    front_clearance = LaunchConfiguration("front_clearance")
    parking_robot_safety_radius = LaunchConfiguration("parking_robot_safety_radius")
    parking_safety_margin = LaunchConfiguration("parking_safety_margin")

    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    safe_navigation_timeout_sec = LaunchConfiguration(
        "safe_navigation_timeout_sec"
    )
    align_timeout_sec = LaunchConfiguration("align_timeout_sec")
    final_yaw_tolerance = LaunchConfiguration("final_yaw_tolerance")

    use_manipulator = LaunchConfiguration("use_manipulator")

    return LaunchDescription(
        [
            # Identity / namespacing.
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/vrpn_mocap/hockey_sticks_1/pose",
            ),

            # QP solver backend for the linearized CLF-CBF-QP controller.
            DeclareLaunchArgument("safe_qp_solver", default_value="cvxopt"),

            # Dynamic robot obstacles. Leave ids empty to disable.
            DeclareLaunchArgument("safe_dynamic_robot_ids", default_value="[]"),
            DeclareLaunchArgument(
                "safe_dynamic_obstacles_required",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "safe_dynamic_controlled_robot_radius",
                default_value="0.18",
            ),
            DeclareLaunchArgument(
                "safe_dynamic_robot_radius",
                default_value="0.18",
            ),
            DeclareLaunchArgument(
                "safe_dynamic_robot_safety_margin",
                default_value="0.10",
            ),

            # Parking geometry and route planning.
            DeclareLaunchArgument("parking_enabled", default_value="true"),
            DeclareLaunchArgument("cushion_length", default_value="1.0"),
            DeclareLaunchArgument("cushion_width", default_value="0.12"),
            DeclareLaunchArgument("parking_front_axis", default_value="y"),
            DeclareLaunchArgument("front_normal_sign", default_value="-1.0"),
            DeclareLaunchArgument("cushion_obstacle_axis", default_value="local_x"),
            DeclareLaunchArgument("desired_normal_distance", default_value="0.35"),
            DeclareLaunchArgument("parking_lateral_offset", default_value="0.0"),
            DeclareLaunchArgument("pre_park_backoff", default_value="0.40"),
            DeclareLaunchArgument("side_clearance", default_value="0.35"),
            DeclareLaunchArgument("front_clearance", default_value="0.35"),
            DeclareLaunchArgument(
                "parking_robot_safety_radius",
                default_value="0.20",
            ),
            DeclareLaunchArgument("parking_safety_margin", default_value="0.10"),

            # Mission-level speed and timeout knobs.
            DeclareLaunchArgument("linear_speed", default_value="0.4"),
            DeclareLaunchArgument("angular_speed", default_value="0.8"),
            DeclareLaunchArgument(
                "safe_navigation_timeout_sec",
                default_value="30.0",
            ),
            DeclareLaunchArgument("align_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("final_yaw_tolerance", default_value="0.08"),

            # Only enable when arm/gripper dependencies are installed and needed.
            DeclareLaunchArgument("use_manipulator", default_value="false"),

            Node(
                package="hockey_controller",
                executable="navigation_server",
                name="navigation_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="safe_navigation_server",
                name="safe_navigation_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "action_name": "safe_navigate_to_point",
                        "qp_solver": safe_qp_solver,
                        "dynamic_robot_ids": safe_dynamic_robot_ids,
                        "dynamic_obstacles_required": (
                            safe_dynamic_obstacles_required
                        ),
                        "dynamic_controlled_robot_radius": (
                            safe_dynamic_controlled_robot_radius
                        ),
                        "dynamic_robot_radius": safe_dynamic_robot_radius,
                        "dynamic_robot_safety_margin": (
                            safe_dynamic_robot_safety_margin
                        ),
                        "target_pose_topic": target_pose_topic,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="spin_server",
                name="spin_server",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="move_arm_server",
                name="move_arm_server",
                namespace=namespace,
                output="screen",
                condition=IfCondition(use_manipulator),
                parameters=[
                    {
                        "robot_id": robot_id,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="gripper_control_server",
                name="gripper_control_server",
                namespace=namespace,
                output="screen",
                condition=IfCondition(use_manipulator),
                parameters=[
                    {
                        "robot_id": robot_id,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="mission_manager",
                name="mission_manager",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "navigation_action": "navigate_to_point",
                        "safe_navigation_action": "safe_navigate_to_point",
                        "spin_action": "spin",
                        "robot_id": robot_id,
                        "cushion_pose_topic": target_pose_topic,
                        "parking_enabled": parking_enabled,
                        "cushion_length": cushion_length,
                        "cushion_width": cushion_width,
                        "parking_front_axis": parking_front_axis,
                        "front_normal_sign": front_normal_sign,
                        "cushion_obstacle_axis": cushion_obstacle_axis,
                        "desired_normal_distance": desired_normal_distance,
                        "parking_lateral_offset": parking_lateral_offset,
                        "pre_park_backoff": pre_park_backoff,
                        "side_clearance": side_clearance,
                        "front_clearance": front_clearance,
                        "parking_robot_safety_radius": parking_robot_safety_radius,
                        "parking_safety_margin": parking_safety_margin,
                        "linear_speed": linear_speed,
                        "angular_speed": angular_speed,
                        "safe_navigation_timeout_sec": (
                            safe_navigation_timeout_sec
                        ),
                        "align_timeout_sec": align_timeout_sec,
                        "final_yaw_tolerance": final_yaw_tolerance,
                    }
                ],
            ),
        ]
    )
