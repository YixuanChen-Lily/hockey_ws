xhost +

docker exec -it hockey /ros_entrypoint.sh /bin/bash -ic '
  cd /hockey_ws
  exec /bin/bash
'