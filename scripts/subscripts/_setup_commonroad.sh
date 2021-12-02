#!/bin/bash
if [[ -z ${paf_dir+z} ]]; then
  SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
  export paf_dir="$SCRIPT_DIR/../../"
  sudo apt update
  sudo apt upgrade
fi

bash _set_python_executable.sh
pip install commonroad-io
mkdir ~/commonroad-tools 2>/dev/null
cd ~/commonroad-tools || exit 1
conda deactivate 2>/dev/null

# recommended not to install everything at once..
install_search=0
install_route_planner=0
install_driveability_checker=0
#install_collision_checker=0 # deprecated

if ((install_search)); then
  # commonroad search (SMP Motion Planner and Maneuvering - local planner)
  cd ~/commonroad-tools || exit 1
  git clone https://gitlab.lrz.de/tum-cps/commonroad-search.git 2>/dev/null
  cd commonroad-search || exit 1
  git pull
  ln -sfn ~/commonroad-tools/commonroad-search/SMP/ $paf_dir/
  pip install -r requirements.txt --no-dependencies
  sudo apt-get install imagemagick imagemagick-doc -y
fi
if ((install_route_planner)); then
  # commonroad route planner (global planning - no obstacles)
  cd ~/commonroad-tools || exit 1
  git clone https://gitlab.lrz.de/tum-cps/commonroad-route-planner.git 2>/dev/null
  cd commonroad-route-planner || exit 1
  git pull
  pip install . --  || (echo "python install failed" && exit 1)
fi
if ((install_driveability_checker)); then
  # commonroad drivability checker (obstacle avoidance - local planner)
  cd ~/commonroad-tools || exit 1
  pip install pybind11 1>/dev/null
  git clone https://github.com/danfis/libccd 2>/dev/null
  cd libccd || exit 1
  git pull 1>/dev/null
  mkdir build 2>/dev/null
  cd build || exit 1
  cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON -DBUILD_SHARED_LIBS=ON .. || (echo "build failed" && exit 1)
  make || (echo "build failed" && exit 1)
  sudo make install || (echo "build failed" && exit 1)
  cd ~/commonroad-tools || exit 1

  git clone https://github.com/flexible-collision-library/fcl 2>/dev/null
  cd fcl || exit 1
  git pull
  sudo apt-get install libboost-dev libboost-thread-dev libboost-test-dev libboost-filesystem-dev libeigen3-dev -y
  mkdir build 2>/dev/null
  cd build || exit 1
  cmake .. || (echo "build failed" && exit 1)
  make || (echo "build failed" && exit 1)
  sudo make install || (echo "build failed" && exit 1)

  cd ~/commonroad-tools || exit 1
  git clone https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker.git 2>/dev/null
  cd commonroad-drivability-checker || exit 1
  git pull
  pip install -r requirements.txt

  cd third_party/libs11n || exit
  mkdir build 2>/dev/null
  cd build || exit 1
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make
  sudo make install

  cd ~/commonroad-tools || exit 1
  cd commonroad-drivability-checker || exit 1
  py_version="$(echo $(python --version) | cut -d' ' -f 2)"
  bash build.sh -e "$(which python)" -v "$py_version" -i -j 2
  sudo python setup.py install
fi
#if ((install_collision_checker)); then # version 2019.1 deprecated !
#  # commonroad collision checker
#  cd ~/commonroad-tools || exit 1
#  git clone https://gitlab.lrz.de/tum-cps/commonroad-collision-checker.git 2>/dev/null
#  cd commonroad-collision-checker || exit 1
#  git pull
#  mkdir build 2>/dev/null
#  cd build || exit 1
#  cmake -DADD_PYTHON_BINDINGS=TRUE -DPYTHON_EXECUTABLE="$(which python)" -DCMAKE_BUILD_TYPE=Release .. || (echo "build failed" && exit 1)
#  make || (echo "build failed" && exit 1)
#  cd ~/commonroad-tools/commonroad-collision-checker || exit 1
#  #  sudo python setup.py install
#  ln -sfn ~/commonroad-tools/commonroad-drivability-checker/ /usr/local/lib/python3.8/dist-packages/
#fi

cd ~/commonroad-tools || exit 1
pip install --upgrade numpy --no-dependencies
