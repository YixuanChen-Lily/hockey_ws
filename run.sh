xhost +

docker run -it --rm \
	--network=host --pid=host --ipc=host \
	--volume .:/hockey_ws:rw \
	--volume "$HOME/.Xauthority:/root/.Xauthority:rw" \
	--env="DISPLAY" \
	--mount type=bind,source=/mnt/wslg/.X11-unix,target=/tmp/.X11-unix \
	--name="hockey" dji_robomaster_ros:1.0 \
	/bin/bash -c "cd /hockey_ws && colcon build --packages-select hockey_interfaces hockey_controller --symlink-install && source install/setup.bash && ros2 pkg prefix hockey_controller && ros2 launch hockey_controller mission.launch.py"
