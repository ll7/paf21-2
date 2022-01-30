#!/bin/bash

find ~/.ros/tensorboard/ -mtime +3 -exec rm -rf {} \;
tensorboard --logdir ~/.ros/tensorboard/
