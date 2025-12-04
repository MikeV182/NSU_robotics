import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('ex0501')
    xacro_path = os.path.join(package_dir, 'ex01', 'robot.urdf.xacro')

    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found at: {xacro_path}")

    try:
        robot_description = subprocess.check_output(['xacro', xacro_path], text=True)
    except Exception as e:
        raise RuntimeError(f"Error processing xacro: {e}")

    rviz_path = os.path.join(package_dir, 'rviz', 'robot.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_path]
        ),
    ])