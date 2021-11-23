#!/bin/bash

main_launch_package="paf_starter"
main_launch_script="paf_starter.launch"
ros_launch_args="town:=Town03 spawn_point:=-80,2,0,0,0,0"

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
  gnome-terminal -- ~/carla_0.9.10.1/CarlaUE4.sh
  ./subscripts/wait_for_window.sh CarlaUE4 close >/dev/null # wait for window to open
}
function start_terminal() { # opt:name, cmd
  if (($# == 2)); then
    gnome-terminal --title=$1 -- "$2"
  else
    gnome-terminal -- $1
  fi
}
function start_terminal_wait_until_it_stays_open() { # cmd, name
  STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
  while [ "$STATUS" = "closed" ]; do
    start_terminal "$1"
    sleep 3
    STATUS=$(./subscripts/wait_for_window.sh "$2" open 3)
  done
}

cd ~/paf21-2/scripts/ || exit
echo "CARLA AND ROS INSTANCE MANAGER (arguments: --skip-carla-restart --build)"
trap exit_program SIGINT

CARLA_SKIP=0
BUILD_ROS=0
for VAR in "$@"; do
  if [ "$VAR" = "--skip-carla-restart" ]; then
    CARLA_SKIP=1
  fi
  if [ "$VAR" = "--build" ]; then
    BUILD_ROS=1
  fi
done

if ((BUILD_ROS)); then
  close_ros
  ./build_ros.sh
  cd ~/paf21-2/scripts/ || exit
fi
if ((CARLA_SKIP)); then
  close_ros
  if carla_available; then
    echo skipping carla restart...
  else
    echo starting carla...
    carla_start
  fi
else
  close_all
  carla_start
fi
echo "starting main launcher..."
start_terminal_wait_until_it_stays_open "roslaunch $main_launch_package $main_launch_script $ros_launch_args" "$main_launch_script"
# gnome-terminal -- rosrun rviz rviz -d ~/paf21-2/paf.cfg.rviz

echo "spawning npcs... (CURRENTLY DEACTIVATED)"
#gnome-terminal --title="spawn_npc.py" -- python ~/carla_0.9.10.1/PythonAPI/examples/spawn_npc.py

echo "done"
echo ""
echo "press ctrl+c to kill all ros terminals."
echo "listening for error/exit of carla environment..."
./subscripts/wait_for_window.sh CarlaUE4 open >/dev/null

# exit all ros instances
echo "closing all ros launchers"
exit_program
