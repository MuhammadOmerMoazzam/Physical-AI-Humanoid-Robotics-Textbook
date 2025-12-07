# capstone/ros2_ws/src/capstone_bringup/launch/capstone_full.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Isaac Sim) or real robot'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.42',
        description='IP address of the real robot'
    )
    
    # Get launch configurations
    use_sim = LaunchConfiguration('use_sim')
    robot_ip = LaunchConfiguration('robot_ip')
    
    # Whisper voice recognition node
    whisper_node = Node(
        package='capstone_bringup',
        executable='whisper_node.py',
        name='whisper_voice_recognition',
        parameters=[{
            'model_size': 'large-v3',
            'language': 'en',
            'device': 'cuda',
        }],
        condition=IfCondition(use_sim)  # Only needed for testing
    )
    
    # Conversational humanoid node (main pipeline)
    conversational_humanoid_node = Node(
        package='capstone_bringup',
        executable='conversational_humanoid_node.py',
        name='conversational_humanoid_node',
        parameters=[{
            'use_simulation': use_sim,
            'robot_ip_address': robot_ip,
            'model_weights_path': 'capstone/models/rdt1b-1.2b-vla-8bit.onnx',
            'control_frequency': 100,  # Hz
            'safety_enabled': True,
        }]
    )
    
    # Navigation and planning node
    nav_planning_node = Node(
        package='capstone_bringup',
        executable='nav_planner_node.py',
        name='navigation_planner',
        parameters=[{
            'planner_type': 'nav2',
            'map_resolution': 0.05,  # meters per pixel
        }]
    )
    
    # Safety supervisor node
    safety_supervisor_node = Node(
        package='capstone_bringup',
        executable='safety_supervisor.py',
        name='safety_supervisor',
        parameters=[{
            'speed_separation_enabled': True,
            'force_limiting_enabled': True,
            'emergency_stop_words': ['stop', 'emergency', 'halt'],
        }]
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_ip_arg,
        whisper_node,
        conversational_humanoid_node,
        nav_planning_node,
        safety_supervisor_node
    ])