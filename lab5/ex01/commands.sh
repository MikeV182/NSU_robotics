colcon build --packages-select my_robot_pkg
source install/setup.bash
ros2 launch my_robot_pkg robot_display.launch.py

# В RViz:
# 
# Fixed Frame: base_link
# RobotModel -> Description Topic: /robot_description
