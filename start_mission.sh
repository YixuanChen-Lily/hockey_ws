xhost +

docker exec -it hockey /ros_entrypoint.sh /bin/bash -ic '
  set -e
  cd /hockey_ws
  ros2 service call /mission/start std_srvs/srv/Trigger {}
  exec /bin/bash
'