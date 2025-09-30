### Create move_to_goal Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake move_to_goal --dependencies rclcpp geometry_msgs turtlesim
```

### Write `move_to_goal.cpp`

### Update CMakeLists.txt
In `move_to_goal/CMakeLists.txt`, add:
```cmake
add_executable(move_to_goal src/move_to_goal.cpp)
ament_target_dependencies(move_to_goal rclcpp geometry_msgs turtlesim)
install(TARGETS move_to_goal DESTINATION lib/${PROJECT_NAME})
```

### Build and Source
```bash
cd ~/ros2_ws
colcon build --packages-select move_to_goal
source install/setup.bash
```

### Run Nodes
1. Terminal 1: `ros2 run turtlesim turtlesim_node`
2. Terminal 2: `ros2 run move_to_goal move_to_goal --ros-args -p x:=5.0 -p y:=5.0 -p theta:=1.57`

### Save Files to current folder
```bash
cp -r ~/ros2_ws/src/move_to_goal .
```
