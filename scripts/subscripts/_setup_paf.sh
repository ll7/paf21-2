#!/bin/bash
sudo apt install -y python-is-python3 wmctrl ros-noetic-cv-bridge

pip install carla_birdeye_view pre-commit catkin_tools

cd ~/paf21-2/ || exit 1

ln -s ~/paf21-2/paf_ros/ ~/carla-ros-bridge/catkin_ws/src/
./scripts/build_ros.sh --clean --no-build || exit 1

pre-commit install
pre-commit install -t post-checkout
pre-commit run --all-files
