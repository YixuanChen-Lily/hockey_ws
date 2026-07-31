#!/usr/bin/env bash
xhost +

docker exec -it hockey /ros_entrypoint.sh /bin/bash -ic '
	cd /hockey_ws
	python3 -m pip install -r src/hockey_controller/requirements.txt
	colcon build --packages-select hockey_interfaces hockey_controller --symlink-install
	source install/setup.bash
	ros2 launch hockey_controller mission.launch.py \
	robot_id:=2 \
	target_pose_topic:=/vrpn_mocap/hockey_sticks_1/pose \
	cushion_length:=0.3 \
	cushion_width:=0.42 \
	parking_front_axis:=x \
	front_normal_sign:=1.0 \
	cushion_obstacle_axis:=local_y \
	desired_normal_distance:=0.55 \
	parking_lateral_offset:=0.10 \
	parking_robot_safety_radius:=0.0 \
	parking_safety_margin:=0.0 \
	safe_dynamic_robot_radius:=0.0 \
	safe_dynamic_robot_safety_margin:=0.0 \
	safe_dynamic_robot_ids:="[]" \
	use_manipulator:=true \
	shooting_enabled:=true \
	shooting_role:=shooter \
	puck_pose_topic:=/vrpn_mocap/hockey_puck_blue/pose \
	goal_pose_topic:=/vrpn_mocap/hockey_goal_1/pose
	exec /bin/bash
'
