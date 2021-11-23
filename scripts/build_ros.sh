#!/bin/bash

clean=0
rosdep=0
build=1
for VAR in "$@"; do
  if [ "$VAR" = "--clean" ]; then
    clean=1
  fi
  if [ "$VAR" = "--rosdep" ]; then
    clean=1
  fi
  if [ "$VAR" = "--no-build" ]; then
    build=0
  fi
done

cd ~/carla-ros-bridge/catkin_ws/ || exit 1
source /opt/ros/noetic/setup.bash
if ((clean)); then
  catkin clean -y
fi
if ((rosdep)); then
  rosdep update
fi

if ((build)); then
  catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
fi

echo "done building..."
