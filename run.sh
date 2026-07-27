#!/usr/bin/env bash

xhost +

docker run -it \
  --network=host \
  --pid=host \
  --ipc=host \
  --volume .:/hockey_ws:rw \
  --volume "$HOME/.Xauthority:/root/.Xauthority:rw" \
  --env DISPLAY \
  --env ROS_DISCOVERY_SERVER=192.168.0.2:11811 \
  --env ROS_SUPER_CLIENT=TRUE \
  --mount type=bind,source=/mnt/wslg/.X11-unix,target=/tmp/.X11-unix \
  --name hockey \
  dji_robomaster_ros:1.0 \
  /bin/bash -lc "cd /hockey_ws && \
    python3 -m pip install -r src/hockey_controller/requirements.txt && \
    colcon build --packages-select hockey_interfaces hockey_controller --symlink-install && \
    source install/setup.bash && \
    ros2 pkg prefix hockey_controller"
  #   ros2 launch hockey_controller mission.launch.py \
  #   robot_id:=9\
  #   target_pose_topic:=/vrpn_mocap/hockey_sticks_4/pose \
	# parking_enabled:=true \
	# safe_qp_solver:=osqp \
	# cushion_length:=0.3 \
	# cushion_width:=0.42 \
	# parking_front_axis:=x \
	# front_normal_sign:=1.0 \
	# cushion_obstacle_axis:=local_y \
	# desired_normal_distance:=0.75 \
	# pre_park_backoff:=0.40 \
	# parking_robot_safety_radius:=0.0 \
	# parking_safety_margin:=0.0"