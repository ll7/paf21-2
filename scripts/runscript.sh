#!/bin/bash

main_launch_package="paf_starter"
main_launch_script="paf_starter.launch"
ros_launch_args="town:=Town03 spawn_point:=-80,2,0,0,0,90 manual_control:=false validation:=true"
npc_launch_args="-n 200 -w 80" # n=vehicles, w=pedestrians

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export paf_dir="$SCRIPT_DIR/../"
bash "$SCRIPT_DIR/subscripts/_set_python_executable.sh"

eval "$(cat ~/.bashrc | tail -n +10)" 1>/dev/null
function carla_available() {
  if [[ "$(wmctrl -l)" =~ "CarlaUE4" ]]; then
    return 0
  else
    return 1
  fi
}
function _close_ros() {
  rosnode kill -a
  wmctrl -c "spawn_npc.py"
  wmctrl -c ".launch"
  wmctrl -c " - RViz"
}
function exit_program() {
  _close_ros
  exit
}
function close_ros() {
  if carla_available; then
    _close_ros
    sleep 3.2 # sleep must be longer than 3 (wait_for_window.sh)
  fi
}
function close_all() {
  if carla_available; then
    wmctrl -c "CarlaUE4"
    _close_ros
    sleep 3.2 # sleep must be longer than 3 (wait_for_window.sh)
  fi
}
function carla_start() {
  echo "bash ~/carla_0.9.10.1/CarlaUE4.sh $1"
  gnome-terminal --title="CarlaUE4" -- ~/carla_0.9.10.1/CarlaUE4.sh "$1"
  ./subscripts/wait_for_window.sh CarlaUE4 close >/dev/null # wait for window to open
  sleep 3
}
function start_terminal() { # opt:name, cmd
  if (($# == 2)); then
    gnome-terminal --title=$1 -- $2
  else
    gnome-terminal -- $1
  fi
}
function start_terminal_wait_until_it_stays_open() { # cmd, name
  echo $1
  STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
  while [ "$STATUS" = "closed" ]; do
    start_terminal "$1"
    sleep 3
    STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
  done
}

cd $paf_dir/scripts/ || exit
echo "CARLA AND ROS INSTANCE MANAGER (arguments: --skip-carla-restart --build --npcs --low-quality)"
trap exit_program SIGINT

CARLA_SKIP=0
BUILD_ROS=0
NPCS=0
CARLA_ARGS=""
for VAR in "$@"; do
  if [ "$VAR" = "--skip-carla-restart" ]; then
    CARLA_SKIP=1
  fi
  if [ "$VAR" = "--build" ]; then
    BUILD_ROS=1
  fi
  if [ "$VAR" = "--npcs" ]; then
    NPCS=1
  fi
  if [ "$VAR" = "--low-quality" ]; then
    CARLA_ARGS="-quality-level=Low"
  fi
done
if [ ! -d "$HOME/carla-ros-bridge/catkin_ws/devel" ]; then
  BUILD_ROS=1
fi
if ((BUILD_ROS)); then
  echo building ros
  ./build_ros.sh --clean
  cd $paf_dir/scripts/ || exit
fi
if ((CARLA_SKIP)); then
  if carla_available; then
    echo skipping carla restart...
  else
    echo starting carla...
    carla_start $CARLA_ARGS
  fi
else
  echo starting carla
  close_all
  carla_start $CARLA_ARGS
fi
eval "$(cat ~/.bashrc | tail -n +10)"
close_ros 2>/dev/null
echo "starting main launcher..."
start_terminal_wait_until_it_stays_open "roslaunch $main_launch_package $main_launch_script $ros_launch_args" "$main_launch_script"

if ((NPCS)); then
  echo "spawning npcs..."
  gnome-terminal --title="spawn_npc.py" -- python ~/carla_0.9.10.1/PythonAPI/examples/spawn_npc.py $npc_launch_args
fi

if (rosnode list | grep ERROR); then
  echo ERROR!!
  exit_program
  exit
fi

echo "loaded the following nodes successfully:"
rosnode list
echo ""
echo "press ctrl+c to kill all ros terminals."

echo "listening for error/exit of carla environment..."
./subscripts/wait_for_window.sh CarlaUE4 open >/dev/null

# exit all ros instances
echo "closing all ros launchers"
exit_program
