# launch/sim2real_walking.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='unitree_g1',
        description='Robot type: unitree_g1, figure_02, etc.'
    )
    
    policy_arg = DeclareLaunchArgument(
        'policy',
        default_value='rdt1b_final.onnx',
        description='Policy file to deploy'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='IP address of the real robot'
    )
    
    enable_dr_arg = DeclareLaunchArgument(
        'enable_dr',
        default_value='false',
        description='Enable domain randomization (no longer needed)'
    )
    
    robot = LaunchConfiguration('robot')
    policy = LaunchConfiguration('policy')
    robot_ip = LaunchConfiguration('robot_ip')
    enable_dr = LaunchConfiguration('enable_dr')
    
    # Sim2Real deployment node
    sim2real_deploy_node = Node(
        package='sim2real',
        executable='deploy_node.py',
        name='sim2real_deploy',
        parameters=[{
            'robot_type': robot,
            'policy_file': policy,
            'robot_ip_address': robot_ip,
            'domain_randomization_enabled': enable_dr,
            'calibration_file': '$(find sim2real)/config/' + robot + '_calibrated_2025.yaml'
        }],
        remappings=[
            ('/simulated_joint_states', '/real_joint_states'),
            ('/simulated_sensor_data', '/real_sensor_data'),
            ('/policy_actions', '/real_robot/joint_group_position_controller/joint_trajectory'),
        ]
    )
    
    # Domain randomization node (if enabled)
    dr_node = Node(
        package='sim2real',
        executable='domain_rand_node.py',
        name='domain_randomization',
        parameters=[{
            'enabled': enable_dr
        }],
        condition=launch.conditions.IfCondition(enable_dr)
    )
    
    # Calibrated dynamics node
    dynamics_node = Node(
        package='sim2real',
        executable='real_dynamics_node.py',
        name='real_dynamics_compensation',
        parameters=[{
            'calibration_file': '$(find sim2real)/config/' + robot + '_calibrated_2025.yaml'
        }]
    )
    
    return LaunchDescription([
        robot_arg,
        policy_arg,
        robot_ip_arg,
        enable_dr_arg,
        sim2real_deploy_node,
        dr_node,
        dynamics_node
    ])