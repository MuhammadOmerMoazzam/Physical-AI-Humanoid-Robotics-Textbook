# launch/spawn_humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('robot', default_value='g1', description='Robot model to use'),
        
        ComposableNodeContainer(
            name='humanoid_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # All heavy nodes live here → single process
            ]
        ),
        
        Node(
            package='ros2_humanoid_template',
            executable='joint_publisher.py',
            name='wave_demo',
            parameters=[{'use_sim': LaunchConfiguration('use_sim')}]
        )
    ])