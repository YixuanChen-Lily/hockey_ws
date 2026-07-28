docker run -d \
  --restart unless-stopped \
  --network=host \
  --pid=host \
  --ipc=host \
  --volume "$(pwd):/hockey_ws:rw" \
  --volume "$HOME/.Xauthority:/root/.Xauthority:rw" \
  --env DISPLAY \
  --mount type=bind,source=/mnt/wslg/.X11-unix,target=/tmp/.X11-unix \
  --name hockey \
  dji_robomaster_ros:1.0 \
  sleep infinity