from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_id = LaunchConfiguration("robot_id")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    safe_target_x = LaunchConfiguration("safe_target_x")
    safe_target_y = LaunchConfiguration("safe_target_y")
    rotations = LaunchConfiguration("rotations")
    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    navigation_timeout_sec = LaunchConfiguration("navigation_timeout_sec")
    safe_navigation_timeout_sec = LaunchConfiguration(
        "safe_navigation_timeout_sec"
    )
    spin_timeout_sec = LaunchConfiguration("spin_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument("target_x", default_value="1.0"),
            DeclareLaunchArgument("target_y", default_value="0.0"),
            DeclareLaunchArgument("safe_target_x", default_value="1.0"),
            DeclareLaunchArgument("safe_target_y", default_value="0.0"),
            DeclareLaunchArgument("rotations", default_value="1"),
            DeclareLaunchArgument("linear_speed", default_value="0.4"),
            DeclareLaunchArgument("angular_speed", default_value="0.8"),
            DeclareLaunchArgument("navigation_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument(
                "safe_navigation_timeout_sec",
                default_value="30.0",
            ),
            DeclareLaunchArgument("spin_timeout_sec", default_value="15.0"),
            Node(
                package="hockey_controller",
                executable="navigation_server",
                name="navigation_server",
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
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "action_name": "safe_navigate_to_point",
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="spin_server",
                name="spin_server",
                output="screen",
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
                output="screen",
                parameters=[
                    {
                        "navigation_action": "navigate_to_point",
                        "safe_navigation_action": "safe_navigate_to_point",
                        "spin_action": "spin",
                        "target_x": target_x,
                        "target_y": target_y,
                        "safe_target_x": safe_target_x,
                        "safe_target_y": safe_target_y,
                        "rotations": rotations,
                        "linear_speed": linear_speed,
                        "angular_speed": angular_speed,
                        "navigation_timeout_sec": navigation_timeout_sec,
                        "safe_navigation_timeout_sec": (
                            safe_navigation_timeout_sec
                        ),
                        "spin_timeout_sec": spin_timeout_sec,
                    }
                ],
            ),
        ]
    )
