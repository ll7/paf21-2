#!/bin/bash

cd ~/carla-ros-bridge/catkin_ws/
source ~/.bashrc
rosdep update
catkin_make
