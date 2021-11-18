#!/bin/bash
sudo apt install -y python-is-python3 wmctrl ros-noetic-cv-bridge

pip install carla_birdeye_view pre-commit

cd ~/paf21-2/ || exit 1
./scripts/build_ros.sh --clean --no-build || exit 1

pre-commit install
pre-commit install -t post-checkout
pre-commit run --all-files
