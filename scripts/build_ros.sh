#!/bin/bash

clean=0
rosdep=0
build=1
fail=0
for VAR in "$@"; do
  if [ "$VAR" = "--clean" ]; then
    clean=1
  fi
  if [ "$VAR" = "--rosdep" ]; then
    rosdep=1
  fi
  if [ "$VAR" = "--no-build" ]; then
    build=0
  fi
done

cd ~/carla-ros-bridge/catkin_ws/ || fail=1
source /opt/ros/noetic/setup.bash || fail=1
if ((clean)); then
  catkin clean -y
  find ~/.ros/log/ -mtime +3 -exec rm -rf {} \;
  find ~/.ros/tensorboard/ -mtime +3 -exec rm -rf {} \;
fi
if ((rosdep)); then
  rosdep update || exit 1
fi

if ((build)); then
  catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 || fail=1
fi

if ((fail)); then
  echo "build failed..."
  exit 1
fi
echo "done building..."
