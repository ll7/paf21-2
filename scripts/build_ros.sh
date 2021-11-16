#!/bin/bash

cd ~/carla-ros-bridge/catkin_ws/
source /opt/ros/noetic/setup.bash
source ~/.bashrc
rosdep update
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
