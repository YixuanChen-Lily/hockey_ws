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
  exec /bin/bash
'