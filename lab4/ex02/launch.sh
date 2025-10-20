#!/bin/bash
cd ~/ros2_ws
colcon build --packages-select ex0402a
source install/setup.bash
ros2 launch ex0402a turtle_carrot.launch.py radius:=3.0 direction_of_rotation:=-1
