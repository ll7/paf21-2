#!/bin/bash
# the commonroad scenario designer is used for map editing only and therefore not needed at runtime

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
export paf_dir="$SCRIPT_DIR/../"
export LD_LIBRARY_PATH=/lib:/usr/lib:/usr/local/lib
bash "$SCRIPT_DIR/subscripts/_set_python_executable.sh"
lanelet_py_fix="        if self._buffered_strtee is None:\n            self._create_buffered_strtree()"
lanelet_py=~/.local/lib/python3.8/site-packages/commonroad/scenario/lanelet.py
if (which crdesigner); then
  fixed=$(cat $lanelet_py | grep -q "if self._buffered_strtee is None")
  if (! $fixed); then
    echo "sudo required for fixing commonroad.scenario.lanelet.py (line 1423)"
    match="                                                                'type = {}'.format(type(shape))"
    sudo cp -n $lanelet_py "$lanelet_py.original"
    sudo sed -i "s/$match/$match\n$lanelet_py_fix/" $lanelet_py
    exit
  fi
  if [ $# -eq 0 ]; then
    crdesigner "$@"
  else
    crdesigner -i ~/.ros/maps/Rules/"$1".xml
  fi
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
