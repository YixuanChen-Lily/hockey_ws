#!/usr/bin/env bash
xhost +

docker exec -it hockey /bin/bash -lc '
  if [ -d /opt/ros ]; then
    for f in /opt/ros/*/setup.bash; do
      if [ -f "$f" ]; then
        source "$f"
        break
      fi
    done
  fi
  cd /hockey_ws
  python3 -m pip install -r src/hockey_controller/requirements.txt
  colcon build --packages-select hockey_interfaces hockey_controller --symlink-install
  source install/setup.bash
  ros2 pkg prefix hockey_controller
  ros2 launch hockey_controller mission.launch.py \
  robot_id:=9 \
  target_pose_topic:=/vrpn_mocap/hockey_sticks_1/pose \
	safe_qp_solver:=osqp \
	cushion_length:=0.3 \
	cushion_width:=0.42 \
	parking_front_axis:=x \
	front_normal_sign:=1.0 \
	cushion_obstacle_axis:=local_y \
	desired_normal_distance:=0.75 \
	pre_park_backoff:=0.40 \
	parking_robot_safety_radius:=0.0 \
	parking_safety_margin:=0.0
  exec /bin/bash
'
