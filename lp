#!/bin/bash
rosnode kill paf_local_planner 2>/dev/null
roslaunch paf_planning paf_local_planner.launch
