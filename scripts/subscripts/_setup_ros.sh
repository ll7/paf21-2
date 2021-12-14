#!/bin/bash
set -e
# ROS Noetic
# http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
if grep -qR "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
  echo "skip .bashrc additionals"
else
  echo "source /opt/ros/noetic/setup.bash" >>~/.bashrc
fi
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
