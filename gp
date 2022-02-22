#!/bin/bash
rosnode kill paf_global_planner 2>/dev/null
roslaunch paf_planning paf_global_planner.launch
