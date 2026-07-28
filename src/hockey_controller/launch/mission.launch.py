from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_id = LaunchConfiguration("robot_id")
    safe_lookahead_distance = LaunchConfiguration("safe_lookahead_distance")
    safe_point_gain = LaunchConfiguration("safe_point_gain")
    safe_clf_gain = LaunchConfiguration("safe_clf_gain")
    safe_cbf_gain = LaunchConfiguration("safe_cbf_gain")
    safe_slack_weight = LaunchConfiguration("safe_slack_weight")
    safe_max_point_speed = LaunchConfiguration("safe_max_point_speed")
    safe_obstacles_enabled = LaunchConfiguration("safe_obstacles_enabled")
    safe_obstacle_safe_margin = LaunchConfiguration("safe_obstacle_safe_margin")
    safe_robot_safety_radius = LaunchConfiguration("safe_robot_safety_radius")
    safe_qp_solver = LaunchConfiguration("safe_qp_solver")
    safe_qp_verbose = LaunchConfiguration("safe_qp_verbose")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    safe_target_offset_x = LaunchConfiguration("safe_target_offset_x")
    safe_target_offset_y = LaunchConfiguration("safe_target_offset_y")
    safe_target_orientation_offset = LaunchConfiguration(
        "safe_target_orientation_offset"
    )
    safe_orient_to_target = LaunchConfiguration("safe_orient_to_target")
    safe_use_target_pose = LaunchConfiguration("safe_use_target_pose")
    cushion_length = LaunchConfiguration("cushion_length")
    cushion_width = LaunchConfiguration("cushion_width")
    parking_front_axis = LaunchConfiguration("parking_front_axis")
    front_normal_sign = LaunchConfiguration("front_normal_sign")
    desired_normal_distance = LaunchConfiguration("desired_normal_distance")
    pre_park_backoff = LaunchConfiguration("pre_park_backoff")
    parking_robot_safety_radius = LaunchConfiguration("parking_robot_safety_radius")
    side_clearance = LaunchConfiguration("side_clearance")
    front_clearance = LaunchConfiguration("front_clearance")
    parking_safety_margin = LaunchConfiguration("parking_safety_margin")
    cushion_circle_spacing = LaunchConfiguration("cushion_circle_spacing")
    parking_lookahead_distance = LaunchConfiguration("parking_lookahead_distance")
    final_approach_speed = LaunchConfiguration("final_approach_speed")
    final_approach_point_gain = LaunchConfiguration("final_approach_point_gain")
    align_gain = LaunchConfiguration("align_gain")
    final_yaw_tolerance = LaunchConfiguration("final_yaw_tolerance")
    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    arm_action_name = LaunchConfiguration("arm_action_name")
    driver_arm_action_name = LaunchConfiguration("driver_arm_action_name")
    gripper_action_name = LaunchConfiguration("gripper_action_name")
    driver_gripper_action_name = LaunchConfiguration(
        "driver_gripper_action_name"
    )

    # ================================================================
    # Hockey-stick pickup and ready-position tuning parameters.
    #
    # MoveArm uses meters: +X points
    # forward and +Z points upward. With *_relative=false, X/Z are absolute
    # in arm_base_link; with *_relative=true, they are motion increments.
    # ================================================================
    # Step 1: arm end-effector pose used to reach the hockey stick.
    grab_arm_x = LaunchConfiguration("grab_arm_x")
    grab_arm_z = LaunchConfiguration("grab_arm_z")
    grab_arm_relative = LaunchConfiguration("grab_arm_relative")
    grab_arm_settle_sec = LaunchConfiguration("grab_arm_settle_sec")

    # Step 2: gripper close power and settling time.
    gripper_close_power = LaunchConfiguration("gripper_close_power")
    gripper_close_settle_sec = LaunchConfiguration(
        "gripper_close_settle_sec"
    )

    # Step 3: arm end-effector pose used to lift the captured stick.
    lift_arm_x = LaunchConfiguration("lift_arm_x")
    lift_arm_z = LaunchConfiguration("lift_arm_z")
    lift_arm_relative = LaunchConfiguration("lift_arm_relative")
    lift_arm_settle_sec = LaunchConfiguration("lift_arm_settle_sec")

    # Step 4: open-loop reverse motion using /robotN/cmd_vel Twist.
    backward_distance = LaunchConfiguration("backward_distance")
    backward_duration_sec = LaunchConfiguration("backward_duration_sec")
    backward_publish_rate_hz = LaunchConfiguration(
        "backward_publish_rate_hz"
    )

    # Step 5: arm end-effector pose for the ready-to-hit configuration.
    ready_arm_x = LaunchConfiguration("ready_arm_x")
    ready_arm_z = LaunchConfiguration("ready_arm_z")
    ready_arm_relative = LaunchConfiguration("ready_arm_relative")
    ready_arm_settle_sec = LaunchConfiguration("ready_arm_settle_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument("safe_lookahead_distance", default_value="0.25"),
            DeclareLaunchArgument("safe_point_gain", default_value="0.8"),
            DeclareLaunchArgument("safe_clf_gain", default_value="1.0"),
            DeclareLaunchArgument("safe_cbf_gain", default_value="2.0"),
            DeclareLaunchArgument("safe_slack_weight", default_value="100.0"),
            DeclareLaunchArgument("safe_max_point_speed", default_value="0.4"),
            DeclareLaunchArgument("safe_obstacles_enabled", default_value="true"),
            DeclareLaunchArgument("safe_obstacle_safe_margin", default_value="0.10"),
            DeclareLaunchArgument("safe_robot_safety_radius", default_value="0.20"),
            DeclareLaunchArgument("safe_qp_solver", default_value="osqp"),
            DeclareLaunchArgument("safe_qp_verbose", default_value="false"),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/vrpn_mocap/hockey_sticks_1/pose",
            ),
            DeclareLaunchArgument("safe_target_offset_x", default_value="0.0"),
            DeclareLaunchArgument("safe_target_offset_y", default_value="0.0"),
            DeclareLaunchArgument(
                "safe_target_orientation_offset",
                default_value="0.0",
            ),
            DeclareLaunchArgument("safe_orient_to_target", default_value="true"),
            DeclareLaunchArgument("safe_use_target_pose", default_value="true"),
            DeclareLaunchArgument("cushion_length", default_value="1.0"),
            DeclareLaunchArgument("cushion_width", default_value="0.12"),
            DeclareLaunchArgument("parking_front_axis", default_value="y"),
            DeclareLaunchArgument("front_normal_sign", default_value="-1.0"),
            DeclareLaunchArgument("desired_normal_distance", default_value="0.35"),
            DeclareLaunchArgument("pre_park_backoff", default_value="0.40"),
            DeclareLaunchArgument(
                "parking_robot_safety_radius",
                default_value="0.20",
            ),
            DeclareLaunchArgument("side_clearance", default_value="0.35"),
            DeclareLaunchArgument("front_clearance", default_value="0.35"),
            DeclareLaunchArgument("parking_safety_margin", default_value="0.10"),
            DeclareLaunchArgument("cushion_circle_spacing", default_value="0.20"),
            DeclareLaunchArgument("parking_lookahead_distance", default_value="0.25"),
            DeclareLaunchArgument("final_approach_speed", default_value="0.12"),
            DeclareLaunchArgument("final_approach_point_gain", default_value="0.35"),
            DeclareLaunchArgument("align_gain", default_value="2.0"),
            DeclareLaunchArgument("final_yaw_tolerance", default_value="0.08"),
            DeclareLaunchArgument("linear_speed", default_value="0.4"),
            DeclareLaunchArgument("angular_speed", default_value="0.8"),
            DeclareLaunchArgument(
                "arm_action_name",
                default_value="control_arm",
            ),
            DeclareLaunchArgument(
                "driver_arm_action_name",
                default_value="move_arm",
            ),
            DeclareLaunchArgument(
                "gripper_action_name",
                default_value="control_gripper",
            ),
            DeclareLaunchArgument(
                "driver_gripper_action_name",
                default_value="gripper",
            ),
            # ============================================================
            # Hockey-stick pickup and ready-position parameters.
            #
            # Settle values are waits after Action results. X/Z values must
            # be calibrated on the physical robot.
            # ============================================================
            # Step 1: move the arm to the stick pickup pose.
            DeclareLaunchArgument("grab_arm_x", default_value="0.0"),
            DeclareLaunchArgument("grab_arm_z", default_value="0.0"),
            DeclareLaunchArgument("grab_arm_relative", default_value="false"),
            DeclareLaunchArgument("grab_arm_settle_sec", default_value="0.3"),
            # Step 2: close the gripper (power must remain in [0, 1]).
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
            DeclareLaunchArgument("lift_arm_z", default_value="0.0"),
            DeclareLaunchArgument("lift_arm_relative", default_value="false"),
            DeclareLaunchArgument("lift_arm_settle_sec", default_value="0.3"),
            # Step 4: reverse distance and duration determine Twist speed.
            DeclareLaunchArgument("backward_distance", default_value="0.30"),
            DeclareLaunchArgument(
                "backward_duration_sec",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "backward_publish_rate_hz",
                default_value="20.0",
            ),
            # Step 5: lower the arm into the ready-to-hit pose.
            DeclareLaunchArgument("ready_arm_x", default_value="0.0"),
            DeclareLaunchArgument("ready_arm_z", default_value="0.0"),
            DeclareLaunchArgument("ready_arm_relative", default_value="false"),
            DeclareLaunchArgument("ready_arm_settle_sec", default_value="0.3"),
            Node(
                package="hockey_controller",
                executable="safe_navigation_server",
                name="safe_navigation_server",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "action_name": "safe_navigate_to_point",
                        "lookahead_distance": safe_lookahead_distance,
                        "point_gain": safe_point_gain,
                        "clf_gain": safe_clf_gain,
                        "cbf_gain": safe_cbf_gain,
                        "slack_weight": safe_slack_weight,
                        "max_point_speed": safe_max_point_speed,
                        "obstacles_enabled": safe_obstacles_enabled,
                        "obstacle_safe_margin": safe_obstacle_safe_margin,
                        "robot_safety_radius": safe_robot_safety_radius,
                        "qp_solver": safe_qp_solver,
                        "qp_verbose": safe_qp_verbose,
                        "target_pose_topic": target_pose_topic,
                        "target_offset_x": safe_target_offset_x,
                        "target_offset_y": safe_target_offset_y,
                        "target_orientation_offset": (
                            safe_target_orientation_offset
                        ),
                        "orient_to_target": safe_orient_to_target,
                        "use_target_pose": safe_use_target_pose,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="move_arm_server",
                name="move_arm_server",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "action_name": arm_action_name,
                        "driver_action_name": driver_arm_action_name,
                    }
                ],
            ),
            Node(
                package="hockey_controller",
                executable="gripper_control_server",
                name="gripper_control_server",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "action_name": gripper_action_name,
                        "driver_action_name": driver_gripper_action_name,
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
                        "safe_navigation_action": "safe_navigate_to_point",
                        "arm_action": arm_action_name,
                        "gripper_action": gripper_action_name,
                        "robot_id": robot_id,
                        "cushion_pose_topic": target_pose_topic,
                        "cushion_length": cushion_length,
                        "cushion_width": cushion_width,
                        "parking_front_axis": parking_front_axis,
                        "front_normal_sign": front_normal_sign,
                        "desired_normal_distance": desired_normal_distance,
                        "pre_park_backoff": pre_park_backoff,
                        "parking_robot_safety_radius": parking_robot_safety_radius,
                        "side_clearance": side_clearance,
                        "front_clearance": front_clearance,
                        "parking_safety_margin": parking_safety_margin,
                        "cushion_circle_spacing": cushion_circle_spacing,
                        "parking_lookahead_distance": parking_lookahead_distance,
                        "final_approach_speed": final_approach_speed,
                        "final_approach_point_gain": final_approach_point_gain,
                        "align_gain": align_gain,
                        "final_yaw_tolerance": final_yaw_tolerance,
                        "linear_speed": linear_speed,
                        "angular_speed": angular_speed,
                        "grab_arm_x": grab_arm_x,
                        "grab_arm_z": grab_arm_z,
                        "grab_arm_relative": grab_arm_relative,
                        "grab_arm_settle_sec": grab_arm_settle_sec,
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
        ]
    )
