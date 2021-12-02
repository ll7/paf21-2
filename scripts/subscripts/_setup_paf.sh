#!/bin/bash
if [[ -z ${paf_dir+z} ]]; then
  SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
  export paf_dir="$SCRIPT_DIR/../../"
  sudo apt update
  sudo apt upgrade
fi
bash "$SCRIPT_DIR/_set_python_executable.sh"
sudo apt install -y python-is-python3 wmctrl ros-noetic-cv-bridge

pip install pre-commit catkin_tools commonroad-io #==2021.2
pip install carla_birdeye_view --no-dependencies
pip install --upgrade numpy --no-dependencies

echo "creating symlink for paf_dir=$paf_dir"
cd $paf_dir || exit 1
ln -sfn $paf_dir/paf_ros/ ~/carla-ros-bridge/catkin_ws/src/
echo "setup/cleanup build..."
sleep 1
./scripts/build_ros.sh --clean --no-build || exit 1

echo "installing pre-commit and building ros..."
sleep 1
pre-commit install
pre-commit install -t post-checkout
pre-commit run --all-files
