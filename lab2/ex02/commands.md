### 1. Changed following lines in turtle_frame.cpp inside turtesim package

``` cpp
TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QFrame(parent, f)
  , path_image_(1920, 1200, QImage::Format_ARGB32)
  , path_painter_(&path_image_)
  , frame_count_(0)
  , id_counter_(0)
{
  setFixedSize(1920, 1200);
  setWindowTitle("TurtleSim");
```

### 2. Running the following commands in separate terminals

``` bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
ros2 bag record -o turtle_cmd_vel /turtle1/cmd_vel
```

### 3. Running around in the simulator window using teleop terminal

### 4. Directory created: turtle_cmd_vel. It contains metadata.yaml && *.mcap file

### 5. Extracting pose at 1x && 2x speed with the following commands

``` bash
ros2 run turtlesim turtlesim_node
ros2 topic echo /turtle1/pose > pose_speed_x1.txt
ros2 bag play turtle_cmd_vel_0.mcap

ros2 run turtlesim turtlesim_node
ros2 topic echo /turtle1/pose > pose_speed_x2.txt
ros2 bag play turtle_cmd_vel/turtle_cmd_vel_0.mcap --rate 2
```