from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    robot_id = LaunchConfiguration("robot_id")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    puck_pose_topic = LaunchConfiguration("puck_pose_topic")
    goal_pose_topic = LaunchConfiguration("goal_pose_topic")

    safe_lookahead_distance = LaunchConfiguration("safe_lookahead_distance")
    safe_obstacle_robot_ids = ParameterValue(
        LaunchConfiguration("safe_obstacle_robot_ids"),
        value_type=str,
    )
    safe_obstacle_pose_topics = ParameterValue(
        LaunchConfiguration("safe_obstacle_pose_topics"),
        value_type=str,
    )
    safe_obstacle_pose_radii = ParameterValue(
        LaunchConfiguration("safe_obstacle_pose_radii"),
        value_type=str,
    )
    safe_obstacle_safety_margin = LaunchConfiguration(
        "safe_obstacle_safety_margin"
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
    parking_lookahead_distance = LaunchConfiguration("parking_lookahead_distance")
    final_approach_speed = LaunchConfiguration("final_approach_speed")

    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    pose_timeout_sec = LaunchConfiguration("pose_timeout_sec")
    obstacle_pose_timeout_sec = LaunchConfiguration(
        "obstacle_pose_timeout_sec"
    )
    safe_navigation_timeout_sec = LaunchConfiguration(
        "safe_navigation_timeout_sec"
    )
    spin_timeout_sec = LaunchConfiguration("spin_timeout_sec")
    action_wait_timeout_sec = LaunchConfiguration("action_wait_timeout_sec")
    align_timeout_sec = LaunchConfiguration("align_timeout_sec")
    final_yaw_tolerance = LaunchConfiguration("final_yaw_tolerance")

    shooting_enabled = LaunchConfiguration("shooting_enabled")
    shooting_role = LaunchConfiguration("shooting_role")
    team_name = LaunchConfiguration("team_name")
    teammate_robot_id = LaunchConfiguration("teammate_robot_id")
    team_wait_timeout_sec = LaunchConfiguration("team_wait_timeout_sec")
    shooting_offset_x = LaunchConfiguration("shooting_offset_x")
    shooting_offset_y = LaunchConfiguration("shooting_offset_y")
    shooting_target_radius = LaunchConfiguration("shooting_target_radius")
    shooting_approach_distance = LaunchConfiguration("shooting_approach_distance")
    shooting_contact_gap = LaunchConfiguration("shooting_contact_gap")
    shooting_center_to_puck_distance = LaunchConfiguration(
        "shooting_center_to_puck_distance"
    )
    shooting_spin_direction = LaunchConfiguration("shooting_spin_direction")
    shooting_puck_obstacle_enabled = LaunchConfiguration(
        "shooting_puck_obstacle_enabled"
    )
    shooting_puck_obstacle_radius = LaunchConfiguration(
        "shooting_puck_obstacle_radius"
    )
    avoid_puck_during_align = LaunchConfiguration("avoid_puck_during_align")
    align_puck_angle_margin_deg = LaunchConfiguration("align_puck_angle_margin_deg")
    shooting_angle_offset = LaunchConfiguration("shooting_angle_offset")
    shooting_linear_speed = LaunchConfiguration("shooting_linear_speed")
    shooting_angular_speed = LaunchConfiguration("shooting_angular_speed")
    shooting_spin_rotations = LaunchConfiguration("shooting_spin_rotations")
    shooting_spin_angle_deg = LaunchConfiguration("shooting_spin_angle_deg")
    shooting_timeout_sec = LaunchConfiguration("shooting_timeout_sec")
    shooting_pose_timeout_sec = LaunchConfiguration("shooting_pose_timeout_sec")
    shooting_max_attempts = LaunchConfiguration("shooting_max_attempts")
    use_manipulator = LaunchConfiguration("use_manipulator")
    reset_arm_x = LaunchConfiguration("reset_arm_x")
    reset_arm_z = LaunchConfiguration("reset_arm_z")
    reset_arm_settle_sec = LaunchConfiguration("reset_arm_settle_sec")
    grab_arm_x = LaunchConfiguration("grab_arm_x")
    grab_arm_z = LaunchConfiguration("grab_arm_z")
    grab_arm_relative = LaunchConfiguration("grab_arm_relative")
    grab_arm_settle_sec = LaunchConfiguration("grab_arm_settle_sec")
    gripper_open_power = LaunchConfiguration("gripper_open_power")
    gripper_open_settle_sec = LaunchConfiguration("gripper_open_settle_sec")
    gripper_close_power = LaunchConfiguration("gripper_close_power")
    gripper_close_settle_sec = LaunchConfiguration("gripper_close_settle_sec")
    lift_arm_x = LaunchConfiguration("lift_arm_x")
    lift_arm_z = LaunchConfiguration("lift_arm_z")
    lift_arm_relative = LaunchConfiguration("lift_arm_relative")
    lift_arm_settle_sec = LaunchConfiguration("lift_arm_settle_sec")
    backward_distance = LaunchConfiguration("backward_distance")
    backward_duration_sec = LaunchConfiguration("backward_duration_sec")
    backward_publish_rate_hz = LaunchConfiguration("backward_publish_rate_hz")
    ready_arm_x = LaunchConfiguration("ready_arm_x")
    ready_arm_z = LaunchConfiguration("ready_arm_z")
    ready_arm_relative = LaunchConfiguration("ready_arm_relative")
    ready_arm_settle_sec = LaunchConfiguration("ready_arm_settle_sec")
    plotter_enabled = LaunchConfiguration("plotter_enabled")
    plotter_show_gui = LaunchConfiguration("plotter_show_gui")
    plotter_output_path = LaunchConfiguration("plotter_output_path")

    return LaunchDescription(
        [
            # Identity / namespacing.
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/vrpn_mocap/hockey_sticks_1/pose",
            ),
            DeclareLaunchArgument(
                "puck_pose_topic",
                default_value="/vrpn_mocap/puck/pose",
            ),
            DeclareLaunchArgument(
                "goal_pose_topic",
                default_value="/vrpn_mocap/goal/pose",
            ),

            DeclareLaunchArgument("safe_lookahead_distance", default_value="0.25"),

            # Pose-updated obstacles. Leave lists empty to disable.
            DeclareLaunchArgument("safe_obstacle_robot_ids", default_value="[]"),
            DeclareLaunchArgument("safe_obstacle_pose_topics", default_value="[]"),
            DeclareLaunchArgument("safe_obstacle_pose_radii", default_value="[]"),
            DeclareLaunchArgument("safe_obstacle_safety_margin", default_value="0.0"),

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
            DeclareLaunchArgument("parking_lookahead_distance", default_value="0.25"),
            DeclareLaunchArgument("final_approach_speed", default_value="0.12"),

            # Mission-level speed and timeout knobs.
            DeclareLaunchArgument("linear_speed", default_value="0.4"),
            DeclareLaunchArgument("angular_speed", default_value="0.8"),
            DeclareLaunchArgument("pose_timeout_sec", default_value="150.0"),
            DeclareLaunchArgument(
                "obstacle_pose_timeout_sec",
                default_value="150.0",
            ),
            DeclareLaunchArgument(
                "safe_navigation_timeout_sec",
                default_value="150.0",
            ),
            DeclareLaunchArgument(
                "spin_timeout_sec",
                default_value="150.0",
            ),
            DeclareLaunchArgument(
                "action_wait_timeout_sec",
                default_value="150.0",
            ),
            DeclareLaunchArgument("align_timeout_sec", default_value="150.0"),
            DeclareLaunchArgument("final_yaw_tolerance", default_value="0.08"),

            # Optional shooting phase after parking/pickup.
            DeclareLaunchArgument("shooting_enabled", default_value="false"),
            DeclareLaunchArgument("shooting_role", default_value="single"),
            DeclareLaunchArgument("team_name", default_value="team_rocket"),
            DeclareLaunchArgument("teammate_robot_id", default_value="0"),
            DeclareLaunchArgument("team_wait_timeout_sec", default_value="150.0"),
            DeclareLaunchArgument("shooting_offset_x", default_value="0.0"),
            DeclareLaunchArgument("shooting_offset_y", default_value="0.0"),
            DeclareLaunchArgument("shooting_target_radius", default_value="0.20"),
            DeclareLaunchArgument("shooting_approach_distance", default_value="0.05"),
            DeclareLaunchArgument("shooting_contact_gap", default_value="0.0"),
            DeclareLaunchArgument(
                "shooting_center_to_puck_distance",
                default_value="-1.0",
            ),
            DeclareLaunchArgument("shooting_spin_direction", default_value="ccw"),
            DeclareLaunchArgument(
                "shooting_puck_obstacle_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument("shooting_puck_obstacle_radius", default_value="0.10"),
            DeclareLaunchArgument("avoid_puck_during_align", default_value="true"),
            DeclareLaunchArgument(
                "align_puck_angle_margin_deg",
                default_value="12.0",
            ),
            DeclareLaunchArgument("shooting_angle_offset", default_value="0.0"),
            DeclareLaunchArgument("shooting_linear_speed", default_value="0.3"),
            DeclareLaunchArgument("shooting_angular_speed", default_value="3.0"),
            DeclareLaunchArgument("shooting_spin_rotations", default_value="1"),
            DeclareLaunchArgument("shooting_spin_angle_deg", default_value="30.0"),
            DeclareLaunchArgument("shooting_timeout_sec", default_value="150.0"),
            DeclareLaunchArgument(
                "shooting_pose_timeout_sec",
                default_value="150.0",
            ),
            DeclareLaunchArgument("shooting_max_attempts", default_value="20"),

            # Stick pickup is part of this mission.
            DeclareLaunchArgument("use_manipulator", default_value="true"),
            # Reset the arm to this absolute pose before every mission.
            DeclareLaunchArgument("reset_arm_x", default_value="0.0"),
            DeclareLaunchArgument("reset_arm_z", default_value="0.0"),
            DeclareLaunchArgument("reset_arm_settle_sec", default_value="0.5"),
            DeclareLaunchArgument("grab_arm_x", default_value="0.3"),
            DeclareLaunchArgument("grab_arm_z", default_value="0.3"),
            DeclareLaunchArgument("grab_arm_relative", default_value="false"),
            DeclareLaunchArgument("grab_arm_settle_sec", default_value="0.5"),
            # Step 2: open then close the gripper (power must remain in [0, 1]).
            DeclareLaunchArgument(
                "gripper_open_power",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "gripper_open_settle_sec",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "gripper_close_power",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "gripper_close_settle_sec",
                default_value="0.5",
            ),
            # Step 3: lift the arm while retaining the closed gripper.
            DeclareLaunchArgument("lift_arm_x", default_value="0.0"),
            DeclareLaunchArgument("lift_arm_z", default_value="1.0"),
            DeclareLaunchArgument("lift_arm_relative", default_value="false"),
            DeclareLaunchArgument("lift_arm_settle_sec", default_value="0.5"),
            DeclareLaunchArgument("backward_distance", default_value="0.60"),
            DeclareLaunchArgument(
                "backward_duration_sec",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "backward_publish_rate_hz",
                default_value="20.0",
            ),
            # Step 5: lower the arm into the ready-to-hit pose.
            DeclareLaunchArgument("ready_arm_x", default_value="0.1"),
            DeclareLaunchArgument("ready_arm_z", default_value="0.0"),
            DeclareLaunchArgument("ready_arm_relative", default_value="false"),
            DeclareLaunchArgument("ready_arm_settle_sec", default_value="0.5"),
            DeclareLaunchArgument("plotter_enabled", default_value="false"),
            DeclareLaunchArgument("plotter_show_gui", default_value="false"),
            DeclareLaunchArgument(
                "plotter_output_path",
                default_value="/hockey_ws/src/hockey_controller/parking_plot.png",
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
                        "lookahead_distance": safe_lookahead_distance,
                        "obstacle_robot_ids": safe_obstacle_robot_ids,
                        "obstacle_pose_topics": safe_obstacle_pose_topics,
                        "obstacle_pose_radii": safe_obstacle_pose_radii,
                        "obstacle_pose_safety_margin": (
                            safe_obstacle_safety_margin
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
                        "pose_timeout_sec": pose_timeout_sec,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="shooting_server",
                name="shooting_server",
                namespace=namespace,
                output="screen",
                condition=IfCondition(shooting_enabled),
                parameters=[
                    {
                        "robot_id": robot_id,
                        "puck_pose_topic": puck_pose_topic,
                        "goal_pose_topic": goal_pose_topic,
                        "safe_navigation_action": "safe_navigate_to_point",
                        "spin_action": "spin",
                        "pose_timeout_sec": pose_timeout_sec,
                        "action_wait_timeout_sec": action_wait_timeout_sec,
                        "align_timeout_sec": align_timeout_sec,
                        "safe_lookahead_distance": safe_lookahead_distance,
                        "shooting_center_to_puck_distance": (
                            shooting_center_to_puck_distance
                        ),
                        "shooting_puck_obstacle_radius": (
                            shooting_puck_obstacle_radius
                        ),
                        "avoid_puck_during_align": avoid_puck_during_align,
                        "align_puck_angle_margin_deg": (
                            align_puck_angle_margin_deg
                        ),
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
                        "safe_navigation_action": "safe_navigate_to_point",
                        "spin_action": "spin",
                        "robot_id": robot_id,
                        "cushion_pose_topic": target_pose_topic,
                        "goal_pose_topic": goal_pose_topic,
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
                        "parking_lookahead_distance": parking_lookahead_distance,
                        "final_approach_speed": final_approach_speed,
                        "linear_speed": linear_speed,
                        "angular_speed": angular_speed,
                        "pose_timeout_sec": pose_timeout_sec,
                        "safe_navigation_timeout_sec": (
                            safe_navigation_timeout_sec
                        ),
                        "spin_timeout_sec": spin_timeout_sec,
                        "action_wait_timeout_sec": action_wait_timeout_sec,
                        "align_timeout_sec": align_timeout_sec,
                        "final_yaw_tolerance": final_yaw_tolerance,
                        "shooting_action": "shoot_puck",
                        "shooting_enabled": shooting_enabled,
                        "shooting_role": shooting_role,
                        "team_name": team_name,
                        "teammate_robot_id": teammate_robot_id,
                        "team_wait_timeout_sec": team_wait_timeout_sec,
                        "shooting_offset_x": shooting_offset_x,
                        "shooting_offset_y": shooting_offset_y,
                        "shooting_target_radius": shooting_target_radius,
                        "shooting_approach_distance": (
                            shooting_approach_distance
                        ),
                        "shooting_contact_gap": shooting_contact_gap,
                        "shooting_spin_direction": shooting_spin_direction,
                        "shooting_angle_offset": shooting_angle_offset,
                        "shooting_linear_speed": shooting_linear_speed,
                        "shooting_angular_speed": shooting_angular_speed,
                        "shooting_spin_rotations": shooting_spin_rotations,
                        "shooting_spin_angle_deg": shooting_spin_angle_deg,
                        "shooting_timeout_sec": shooting_timeout_sec,
                        "shooting_max_attempts": shooting_max_attempts,
                        "use_manipulator": use_manipulator,
                        "reset_arm_x": reset_arm_x,
                        "reset_arm_z": reset_arm_z,
                        "reset_arm_settle_sec": reset_arm_settle_sec,
                        "grab_arm_x": grab_arm_x,
                        "grab_arm_z": grab_arm_z,
                        "grab_arm_relative": grab_arm_relative,
                        "grab_arm_settle_sec": grab_arm_settle_sec,
                        "gripper_open_power": gripper_open_power,
                        "gripper_open_settle_sec": gripper_open_settle_sec,
                        "gripper_close_power": gripper_close_power,
                        "gripper_close_settle_sec": gripper_close_settle_sec,
                        "lift_arm_x": lift_arm_x,
                        "lift_arm_z": lift_arm_z,
                        "lift_arm_relative": lift_arm_relative,
                        "lift_arm_settle_sec": lift_arm_settle_sec,
                        "backward_distance": backward_distance,
                        "backward_duration_sec": backward_duration_sec,
                        "backward_publish_rate_hz": backward_publish_rate_hz,
                        "ready_arm_x": ready_arm_x,
                        "ready_arm_z": ready_arm_z,
                        "ready_arm_relative": ready_arm_relative,
                        "ready_arm_settle_sec": ready_arm_settle_sec,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="parking_plotter",
                name="parking_plotter",
                namespace=namespace,
                output="screen",
                condition=IfCondition(plotter_enabled),
                parameters=[
                    {
                        "robot_id": robot_id,
                        "show_gui": plotter_show_gui,
                        "output_path": plotter_output_path,
                        "puck_pose_topic": puck_pose_topic,
                        "goal_pose_topic": goal_pose_topic,
                        "pose_timeout_sec": pose_timeout_sec,
                        "obstacle_pose_timeout_sec": (
                            obstacle_pose_timeout_sec
                        ),
                        "shooting_role": shooting_role,
                        "shooting_offset_x": shooting_offset_x,
                        "shooting_offset_y": shooting_offset_y,
                        "shooting_target_radius": shooting_target_radius,
                        "shooting_contact_gap": shooting_contact_gap,
                        "shooting_center_to_puck_distance": (
                            shooting_center_to_puck_distance
                        ),
                        "shooting_spin_direction": shooting_spin_direction,
                        "shooting_puck_obstacle_enabled": (
                            shooting_puck_obstacle_enabled
                        ),
                        "shooting_puck_obstacle_radius": (
                            shooting_puck_obstacle_radius
                        ),
                        "safe_lookahead_distance": safe_lookahead_distance,
                        "obstacle_robot_ids": safe_obstacle_robot_ids,
                        "obstacle_pose_topics": safe_obstacle_pose_topics,
                        "obstacle_pose_radii": safe_obstacle_pose_radii,
                        "obstacle_robot_safety_margin": (
                            safe_obstacle_safety_margin
                        ),
                    }
                ],
            ),
        ]
    )
