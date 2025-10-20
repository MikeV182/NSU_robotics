#!/bin/bash

cd ~/ros2_ws
colcon build --packages-select ex0403
source install/setup.bash
ros2 launch ex0403 turtle_follower.launch.py
