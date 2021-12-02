#!/bin/bash
# the commonroad scenario designer is used for map editing only and therefore not needed at runtime

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
export paf_dir="$SCRIPT_DIR/../"
export LD_LIBRARY_PATH=/lib:/usr/lib:/usr/local/lib
bash "$SCRIPT_DIR/subscripts/_set_python_executable.sh"
if (which crdesigner); then
  gnome-terminal -- crdesigner "$@"
  exit
fi

mkdir ~/commonroad-tools 2>/dev/null
cd ~/commonroad-tools || exit 1

if [ ! -d "$HOME/commonroad-tools/PROJ/" ]; then
  sudo apt install sqlite3
  git clone https://github.com/OSGeo/PROJ
  cd PROJ || exit 1
  ./autogen.sh
  ./configure
  make
  sudo make install
  make check
fi

cd ~/commonroad-tools || exit 1

if [ ! -d "$HOME/commonroad-tools/commonroad-scenario-designer/" ]; then
  git clone https://gitlab.lrz.de/tum-cps/commonroad-scenario-designer
fi
sudo pip install cartopy
cd commonroad-scenario-designer || exit 1
pip install -r requirements.txt
python -m pip install -U matplotlib
sudo pip install -e .

cd ~/.local/share/proj/ || exit 1
wget --mirror https://cdn.proj.org/

echo "commonroad-designer is now installed and will be launched shortly."
echo ""
sleep 5
gnome-terminal -- crdesigner
