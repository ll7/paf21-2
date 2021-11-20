#!/bin/bash

NUM=0
if [[ "$2" = "open" ]]; then
  while [[ "$(wmctrl -l)" =~ $1 ]]
  do
    if [ $# -eq 3 ]; then
      NUM=$((NUM+1))
      if (( NUM >= $3 )); then
        echo "opened"
        exit
      fi
    fi
    sleep 3
  done
  echo "closed"
elif [[ "$2" = "close" ]]; then
  while ! [[ "$(wmctrl -l)" =~ $1 ]]
  do
    if [ $# -eq 3 ]; then
      NUM=$((NUM+1))
      if (( NUM >= $3 )); then
        echo "closed"
        exit
      fi
    fi
    sleep 3
  done
  echo "opened"
else
  echo "Invalid arguments"
  echo "This script waits while a specific window is open/closed, then exits. Window titles can be substrings."
  echo "usage: ./wait_for_window <window_title> <open|close> <optional:max_no_tries>"
fi
