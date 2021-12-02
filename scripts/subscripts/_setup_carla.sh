#!/bin/bash
cd ~/Downloads || exit
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz
mkdir -p -m o+w ~/carla_0.9.10.1
tar -C ~/carla_0.9.10.1 -zxvf CARLA_0.9.10.1.tar.gz
echo "export PYTHONPATH=$PYTHONPATH:~/carla_0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg" >>~/.bashrc
sudo apt install -y python3-pip

pip3 install --user pygame numpy
# git
sudo apt install -y git
# python link
sudo ln -s /usr/bin/python3 /usr/bin/python
# carla-ros-bridge
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge || exit
git clone --recurse-submodules --depth 1 --branch 0.9.10.1 https://github.com/carla-simulator/ros-bridge.git
cd catkin_ws/src || exit
ln -s ../../ros-bridge
cd ..
rosdep fix-permissions
rosdep update
rosdep install --from-paths src --ignore-src -r
source /opt/ros/noetic/setup.bash
catkin_make
if grep -qR "/opt/ros/noetic/share/rosbash/rosbash" ~/.bashrc; then
  echo "skip .bashrc additionals"
else
  echo "source ~/carla-ros-bridge/catkin_ws/devel/setup.bash" >>~/.bashrc
  echo "source /opt/ros/noetic/share/rosbash/rosbash" >>~/.bashrc
fi
source ~/.bashrc
