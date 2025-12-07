# launch/rdt1b_full_task.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='rdt1b-1.2b-8bit',
        description='RDT-1B model variant to use'
    )
    
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='battery_insert',
        description='Manipulation task to perform'
    )
    
    model = LaunchConfiguration('model')
    task = LaunchConfiguration('task')
    
    # RDT-1B dexterous manipulation node
    rdt1b_node = Node(
        package='manipulation',
        executable='rdt1b_node.py',
        name='rdt1b_full_task_node',
        parameters=[{
            'model_name': model,
            'task_type': task,
            'robot_hand': 'allegro',
            'success_threshold': 0.95,  # 95% success rate target
        }],
        remappings=[
            ('/allegro/joint_commands', '/allegro_hand/joint_effort_trajectory_controller/joint_trajectory'),
        ]
    )
    
    # Isaac Lab simulation bridge
    isaac_lab_bridge = Node(
        package='manipulation',
        executable='isaac_lab_bridge.py',
        name='isaac_lab_bridge_node',
        parameters=[{
            'physics_freq': 4000,  # 4 kHz physics
            'control_freq': 100,   # 100 Hz control
            'num_envs': 1,         # Single environment for real robot
        }]
    )
    
    # Perception bridge for object detection
    perception_bridge = Node(
        package='manipulation',
        executable='perception_bridge.py',
        name='perception_bridge_node',
        parameters=[{
            'camera_topic': '/camera/depth/color/points',
            'detection_model': 'foundationpose',
        }]
    )
    
    return LaunchDescription([
        model_arg,
        task_arg,
        rdt1b_node,
        isaac_lab_bridge,
        perception_bridge
    ])