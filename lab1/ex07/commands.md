ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'Leonardo'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 4.0, y: 2.0, theta: 0.0, name: 'Raphael'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 4.0, theta: 0.0, name: 'Donatello'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 4.0, y: 4.0, theta: 0.0, name: 'Michelangelo'}"

ros2 param set /turtlesim background_g 124

ros2 param get /turtlesim background_g
