"""Launch file for humanoid control nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_humanoid_template',
            executable='joint_publisher',
            name='joint_publisher',
            output='screen',
            parameters=[]
        ),
        Node(
            package='ros2_humanoid_template',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[]
        )
    ])