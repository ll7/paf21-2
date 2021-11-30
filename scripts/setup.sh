#!/bin/bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export paf_dir="$SCRIPT_DIR/../"
bash subscripts/_setup_ros.sh
bash subscripts/_setup_carla.sh
bash subscripts/_setup_paf.sh
bash build_ros.sh
