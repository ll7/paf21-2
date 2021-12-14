#!/bin/bash

main_launch_package="paf_starter"
main_launch_script="paf_starter.launch"
#ros_launch_args="spawn_point:=199,9.5,0,0,0,0 validation:=true"
ros_launch_args="validation:=true"
npc_launch_args="-n 50 -w 50" # n=vehicles, w=pedestrians

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
export paf_dir="$SCRIPT_DIR/../"
bash "$SCRIPT_DIR/subscripts/_set_python_executable.sh" 1>/dev/null

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
  wmctrl -c "rqt_console"
  #  wmctrl -c ".launch"
  wmctrl -c " - RViz"
}
function exit_program() {
  # exit all ros instances
  echo "closing all ros launchers..."
  _close_ros 1>/dev/null
  echo ""
  echo "following log files have been created:"
  echo ""
  cd ~/.ros/log/latest || exit_program
  # shellcheck disable=SC2162
  # shellcheck disable=SC2088
  # shellcheck disable=SC2185
  find -iname "*.log" | tr " " "\n" | while read line; do echo "~/.ros/log/latest/${line:2}"; done
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
  sleep 10
}
function start_terminal() { # opt:name, cmd
  if (($# == 2)); then
    gnome-terminal --title=$1 -- $2
  else
    gnome-terminal -- $1
  fi
}
function reduce_ros_log_noise() {
  # rqt_console, #rqt_logger_level
  rosservice call /rviz/set_logger_level "logger: 'ros'
level: 'Error'"
  rosservice call /carla_ros_bridge/set_logger_level "logger: 'rosout'
level: 'Error'"
  rosservice call /carla_ego_vehicle_ego_vehicle/set_logger_level "logger: 'rosout'
level: 'Error'"
}
function start_terminal_wait_until_it_stays_open() { # cmd, name
  echo $1
  i=0
  STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
  while [ "$STATUS" = "closed" ]; do
    i=(i+1)
    start_terminal "$1"
    sleep 3
    STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
    if ((i > 3)); then
      echo "unable to start $2"
      sleep 30
      exit_program
    fi
  done
}

cd $paf_dir/scripts/ || exit
echo "CARLA AND ROS INSTANCE MANAGER
arguments:
--skip-carla-restart/-scr
--build/-b
--npcs/-n
--low-quality/-lq
--manual-control/-mc
TownXX

Allowed towns are Town01, Town02, Town03, Town04, Town05, Town06, Town07 and Town10HD"

trap exit_program SIGINT

CARLA_SKIP=0
BUILD_ROS=0
NPCS=0
CARLA_ARGS=""
TOWN_ARGS="town:=Town03"
for VAR in "$@"; do
  if [ "$VAR" = "--skip-carla-restart" ]; then
    CARLA_SKIP=1
  fi
  if [ "$VAR" = "-scr" ]; then
    CARLA_SKIP=1
  fi
  if [ "$VAR" = "--manual-control" ]; then
    ros_launch_args="$ros_launch_args manual_control:=true"
  fi
  if [ "$VAR" = "-mc" ]; then
    ros_launch_args="$ros_launch_args manual_control:=true"
  fi
  if [ "$VAR" = "--build" ]; then
    BUILD_ROS=1
  fi
  if [ "$VAR" = "-b" ]; then
    BUILD_ROS=1
  fi
  if [ "$VAR" = "--npcs" ]; then
    NPCS=1
  fi
  if [ "$VAR" = "-n" ]; then
    NPCS=1
  fi
  if [ "$VAR" = "--low-quality" ]; then
    CARLA_ARGS="-quality-level=Low"
  fi
  if [ "$VAR" = "-lq" ]; then
    CARLA_ARGS="-quality-level=Low"
  fi
  if [[ "$VAR" = *"Town"* ]]; then
    TOWN_ARGS="town:=$VAR"
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
    NPCS=0
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
start_terminal_wait_until_it_stays_open "roslaunch $main_launch_package $main_launch_script $TOWN_ARGS $ros_launch_args" "$main_launch_script"
reduce_ros_log_noise
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
reduce_ros_log_noise
gnome-terminal --title="rqt_console" -- rqt_console
echo ""
echo "press ctrl+c to kill all ros terminals."

echo "listening for error/exit of carla environment..."
./subscripts/wait_for_window.sh CarlaUE4 open >/dev/null
exit_program
