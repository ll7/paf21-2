#!/bin/bash

main_launch_package="paf_starter"
main_launch_script="paf_starter.launch"
npc_launch_args="-n 50 -w 50" # n=vehicles, w=pedestrians

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
export paf_dir="$SCRIPT_DIR/../"
bash "$SCRIPT_DIR/subscripts/_set_python_executable.sh" 1>/dev/null

cp -fr "$paf_dir/paf_ros/rosbridge_config.yaml" ~/carla-ros-bridge/ros-bridge/carla_ros_bridge/config/settings.yaml || exit 1

ln -sfn "$paf_dir/paf_ros/" ~/carla-ros-bridge/catkin_ws/src/
ln -sfn "$paf_dir/Maps/" ~/.ros/

red='\033[0;31m'
green=$'\e[1;32m'
yellow=$'\e[1;33m'
blue=$'\e[1;34m'
magenta=$'\e[1;35m'
cyan=$'\e[1;36m'

function echoc() {
  NC='\033[0m'
  # shellcheck disable=SC2059
  printf "${2}${1}${NC}\n"
}

eval "$(cat ~/.bashrc | tail -n +10)" 1>/dev/null
function carla_available() {
  if [[ "$(wmctrl -l)" =~ "CarlaUE4" ]]; then
    return 0
  else
    return 1
  fi
}
function _close_ros() {
  rosnode kill -a 1>/dev/null 2>/dev/null
  wmctrl -c "spawn_npc.py"
  wmctrl -c "rqt_console"
  #  wmctrl -c ".launch"
  wmctrl -c " - RViz"
}
function exit_program() {
  # exit all ros instances
  echo ""
  echoc "closing all ros launchers..." "$yellow"
  _close_ros
  echo ""
  echoc "following log files have been created:" "$yellow"
  cd ~/.ros/log/latest || exit_program
  # shellcheck disable=SC2162
  # shellcheck disable=SC2088
  # shellcheck disable=SC2185
  find -iname "*paf*.log" | tr " " "\n" | while read line; do echo "~/.ros/log/latest/${line:2}"; done
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
  #  python3 ~/carla_0.9.10.1/PythonAPI/util/config.py --delta-seconds 0.02
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
level: 'Error'" 1>/dev/null
  rosservice call /carla_ros_bridge/set_logger_level "logger: 'rosout'
level: 'Error'" 1>/dev/null
  rosservice call /carla_ego_vehicle_ego_vehicle/set_logger_level "logger: 'rosout'
level: 'Error'" 1>/dev/null
}
function start_terminal_wait_until_it_stays_open() { # cmd, name
  echo "$1"
  STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
  while [ "$STATUS" = "closed" ]; do
    start_terminal "$1"
    sleep 3
    STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
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
--no-rules/-nr (rules enabled by default)
--validation/-val
TownXX

Allowed towns are Town01, Town02, Town03, Town04, Town05, Town06, Town07 and Town10HD
"

trap exit_program SIGINT

CARLA_SKIP=0
BUILD_ROS=0
NPCS=0
CARLA_ARGS=""
TOWN_ARGS="town:=Town03"
RULES_ARGS="rules_enabled:=true"
VALIDATION_ARGS="validation:=false"
MANUAL_CTRL_ARGS=""
for VAR in "$@"; do
  if [ "$VAR" = "-h" ]; then
    exit
  fi
  if [ "$VAR" = "--help" ]; then
    exit
  fi
  if [ "$VAR" = "--skip-carla-restart" ]; then
    CARLA_SKIP=1
  fi
  if [ "$VAR" = "-scr" ]; then
    CARLA_SKIP=1
  fi
  if [ "$VAR" = "--no-rules" ]; then
    RULES_ARGS="rules_enabled:=false"
  fi
  if [ "$VAR" = "-nr" ]; then
    RULES_ARGS="rules_enabled:=false"
  fi
  if [ "$VAR" = "--manual-control" ]; then
    MANUAL_CTRL_ARGS="manual_control:=true"
  fi
  if [ "$VAR" = "-mc" ]; then
    MANUAL_CTRL_ARGS="manual_control:=true"
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
  if [ "$VAR" = "--validation" ]; then
    VALIDATION_ARGS="validation:=true"
  fi
  if [ "$VAR" = "-val" ]; then
    VALIDATION_ARGS="validation:=true"
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
if [ ! -f "$HOME/carla-ros-bridge/catkin_ws/devel/setup.bash" ]; then
  BUILD_ROS=1
fi
if ((BUILD_ROS)); then
  echo building ros
  close_all
  ./build_ros.sh --clean || exit 1
  cd "$paf_dir/scripts/" || exit 1
fi
if ((CARLA_SKIP)); then
  if carla_available; then
    echoc "skipping carla restart..." "$yellow"
    echo ""
    NPCS=0
  else
    echoc "starting carla..." "$yellow"
    carla_start $CARLA_ARGS
  fi
else
  echo starting carla
  close_all
  carla_start $CARLA_ARGS
fi
eval "$(cat ~/.bashrc | tail -n +10)"
close_ros 2>/dev/null
echoc "starting main launcher..." "$yellow"
echoc "\nWARNING: $VALIDATION_ARGS" "$red"
echo "the car will only start driving during validation mode (-val) or with the competition manager enabled!"
echo ""
start_terminal_wait_until_it_stays_open "roslaunch $main_launch_package $main_launch_script $TOWN_ARGS $RULES_ARGS $MANUAL_CTRL_ARGS $VALIDATION_ARGS" "$main_launch_script"
reduce_ros_log_noise
if ((NPCS)); then
  echo "spawning npcs..."
  # shellcheck disable=SC2086
  gnome-terminal --title="spawn_npc.py" -- python ~/carla_0.9.10.1/PythonAPI/examples/spawn_npc.py $npc_launch_args
fi

if (rosnode list | grep ERROR); then
  echo ERROR!!
  exit_program
  exit
fi

echoc "loaded the following nodes successfully:" "$yellow"
rosnode list
reduce_ros_log_noise
#gnome-terminal --title="rqt_console" -- rqt_console
echo "press ctrl+c to kill all ros terminals."

echo "listening for error/exit of carla environment..."
./subscripts/wait_for_window.sh CarlaUE4 open >/dev/null
exit_program
