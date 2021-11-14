#!/bin/bash

function carla_available() {
  if [[ "$(wmctrl -l)" =~ "CarlaUE4" ]]; then
    return 0
  else
    return 1
  fi
}
function _close_ros() {
  wmctrl -c ".launch"
  wmctrl -c "rviz"
  wmctrl -c "spawn_npc.py"
}
function exit_program(){
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
  gnome-terminal -- ~/carla_0.9.10.1/CarlaUE4.sh --no-rendering
  ./scripts/wait_for_window.sh CarlaUE4 close >/dev/null # wait for window to open
}
function start_terminal() { # opt:name, cmd
  if (($# == 2)); then
    gnome-terminal --title=$1 -- "$2"
  else
    gnome-terminal -- $1
  fi
}
function start_terminal_wait_until_it_stays_open() { # cmd, name
  STATUS=$(./scripts/wait_for_window.sh "$2" open 3)
  while [ "$STATUS" = "closed" ]; do
    start_terminal "$1"
    sleep 3
    STATUS=$(./scripts/wait_for_window.sh "$2" open 3)
  done
}

echo "CARLA AND ROS INSTANCE MANAGER (arguments: --skip-carla-restart)"
trap exit_program SIGINT

CARLA_SKIP=0
for VAR in "$@"; do
  if [ "$VAR" = "--skip-carla-restart" ]; then
    CARLA_SKIP=1
  fi
done

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
start_terminal_wait_until_it_stays_open "roslaunch carla_ros_bridge carla_ros_bridge.launch" "carla_ros_bridge.launch"
gnome-terminal -- rosrun rviz rviz -d /home/julin/paf21-2/view_only.cfg.rviz

echo "spawning npcs..."
gnome-terminal --title="spawn_npc.py" -- python ~/carla_0.9.10.1/PythonAPI/examples/spawn_npc.py

echo "done"
echo ""
echo "press ctrl+c to kill all ros terminals."
echo "listening for error/exit of carla environment..."
./scripts/wait_for_window.sh CarlaUE4 open >/dev/null

# exit all ros instances
echo "closing all ros launchers"
exit_program
