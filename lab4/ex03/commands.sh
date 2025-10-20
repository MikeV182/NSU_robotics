#!/bin/bash
cd ~/ros2_ws
colcon build --packages-select ex0402b
source install/setup.bash
ros2 launch ex0402b ex0402b.launch.py switch_threshold:=1.5

# ros2 run ex0402b keyboard_listener