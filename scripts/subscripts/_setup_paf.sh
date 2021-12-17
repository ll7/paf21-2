#!/bin/bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
if [[ -z ${paf_dir+z} ]]; then
  export paf_dir="$SCRIPT_DIR/../../"
  sudo apt update
  sudo apt upgrade
fi
bash "$SCRIPT_DIR/_set_python_executable.sh"
sudo apt install -y python-is-python3 wmctrl ros-noetic-cv-bridge

pip install pre-commit catkin_tools commonroad-io tensorflow #==2021.2
pip install carla_birdeye_view --no-dependencies
pip install --upgrade numpy --no-dependencies

echo "setting up commonroad..."
sleep 3

bash "$SCRIPT_DIR/_setup_commonroad.sh" || (echo "commonroad install failed" && exit 1)

echo "creating symlinks for paf_dir=$paf_dir and map_dir..."
mkdir -p ~/.ros/
cd "$paf_dir" || exit 1
ln -sfn "$paf_dir/paf_ros/" ~/carla-ros-bridge/catkin_ws/src/
ln -sfn "$paf_dir/maps/" ~/.ros/
echo "setup/cleanup build..."
sleep 1
./scripts/build_ros.sh --clean --no-build || exit 1

echo "installing pre-commit and building ros..."
sleep 1
pre-commit install || (echo "pre commit install failed, for alt method see _setup_paf.sh" && exit 1)
pre-commit install -t post-checkout
pre-commit run --all-files

echo "done installing"
# alt install curl https://pre-commit.com/install-local.py | python
# then add "source ~/.profile" to file ~/.bashrc
