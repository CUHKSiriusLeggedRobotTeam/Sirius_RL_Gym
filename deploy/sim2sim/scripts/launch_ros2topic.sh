#!/bin/bash

sudo bash -c '
source /opt/ros/humble/setup.bash
source ../install/setup.bash
cd ../src/launch
ros2 launch launch_all.py
'