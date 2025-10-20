``` bash
sudo apt-get install ros-jazzy-rviz2 ros-jazzy-turtle-tf2-py ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-turtlesim
source ~/ros2_ws/install/setup.bash
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
ros2 run tf2_tools view_frames
ros2 run turtlesim turtle_teleop_key
ros2 run tf2_ros tf2_echo turtle1 turtle2 > transform.txt
```