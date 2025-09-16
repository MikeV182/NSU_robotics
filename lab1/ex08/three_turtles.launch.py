from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch first TurtleSim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim1',
            namespace='turtle1',
            parameters=[{'background_r': 69, 'background_g': 86, 'background_b': 255}]
        ),

        # Launch second TurtleSim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim2',
            namespace='turtle2',
            parameters=[{'background_r': 150, 'background_g': 200, 'background_b': 100}]
        ),

        # Launch third TurtleSim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim3',
            namespace='turtle3',
            parameters=[{'background_r': 255, 'background_g': 165, 'background_b': 0}]
        ),

        # Spawn turtle1 in the first simulator
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle1/spawn', 'turtlesim/srv/Spawn',
                 '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle1"}'],
            output='screen'
        ),

        # Spawn turtle2 in the second simulator
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle2/spawn', 'turtlesim/srv/Spawn',
                 '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'],
            output='screen'
        ),

        # Spawn turtle3 in the third simulator
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/turtle3/spawn', 'turtlesim/srv/Spawn',
                 '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle3"}'],
            output='screen'
        ),

        # Launch teleop node to control turtle1
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_turtle',
            namespace='turtle1',
            prefix='xterm -e'
        ),

        # Mimic node: turtle2 follows turtle1
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_turtle2',
            remappings=[
                ('/input/pose', '/turtle1/turtle1/pose'),
                ('/output/cmd_vel', '/turtle2/turtle2/cmd_vel')
            ]
        ),

        # Mimic node: turtle3 follows turtle2
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_turtle3',
            remappings=[
                ('/input/pose', '/turtle2/turtle2/pose'),
                ('/output/cmd_vel', '/turtle3/turtle3/cmd_vel')
            ]
        )
    ])
