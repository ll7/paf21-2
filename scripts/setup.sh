#!/bin/bash

sudo apt update
sudo apt upgrade
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export paf_dir="$SCRIPT_DIR/../"
bash subscripts/_setup_ros.sh
bash subscripts/_setup_carla.sh
bash subscripts/_setup_paf.sh
# bash subscripts/_setup_commonroad.sh
bash build_ros.sh
