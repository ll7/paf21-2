#!/bin/bash

find ~/.ros/tensorboard -mtime +2 -exec rm -rf {} \;
tensorboard --logdir ~/.ros/tensorboard/
