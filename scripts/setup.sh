#!/bin/bash

bash subscripts/_setup_ros.sh
bash subscripts/_setup_carla.sh
bash subscripts/_setup_paf.sh
bash build_ros.sh
