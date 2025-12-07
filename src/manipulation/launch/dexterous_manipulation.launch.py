# launch/dexterous_manipulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='rdt1b-1.2b-8bit',
        description='Model to use: rdt1b-1.2b-8bit, octo, diffusion_policy'
    )
    
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='battery_insert',
        description='Manipulation task: battery_insert, pen_reorient, bottle_pick, etc.'
    )
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='allegro',
        description='Robot hand: allegro, shadow, unitree_z1'
    )
    
    model = LaunchConfiguration('model')
    task = LaunchConfiguration('task')
    robot = LaunchConfiguration('robot')
    
    # RDT-1B node
    rdt1b_node = Node(
        package='manipulation',
        executable='rdt1b_node.py',
        name='rdt1b_manipulation_node',
        parameters=[{
            'model_name': model,
            'task_type': task,
            'robot_hand': robot,
        }],
        condition=IfCondition(
            LaunchConfiguration('model', default='rdt1b-1.2b-8bit')
        )
    )
    
    # Octo inference node
    octo_node = Node(
        package='manipulation',
        executable='octo_infer.py',
        name='octo_manipulation_node',
        parameters=[{
            'model_name': 'octo-400m',
            'task_type': task,
            'robot_hand': robot,
        }],
        condition=IfCondition(
            LaunchConfiguration('model', default='octo')
        )
    )
    
    # Diffusion policy node
    diffusion_node = Node(
        package='manipulation',
        executable='diffusion_policy_node.py',
        name='diffusion_manipulation_node',
        parameters=[{
            'model_name': 'diffusion-policy',
            'task_type': task,
            'robot_hand': robot,
        }],
        condition=IfCondition(
            LaunchConfiguration('model', default='diffusion_policy')
        )
    )
    
    # Isaac Lab simulation bridge
    isaac_lab_bridge = Node(
        package='manipulation',
        executable='isaac_lab_bridge.py',
        name='isaac_lab_bridge',
        parameters=[{
            'sim_freq': 4000,  # 4 kHz
            'control_freq': 100,  # 100 Hz
        }]
    )
    
    return LaunchDescription([
        model_arg,
        task_arg,
        robot_arg,
        rdt1b_node,
        octo_node,
        diffusion_node,
        isaac_lab_bridge
    ])