# launch/walking_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='g1',
        description='Robot model to use (g1, h1, etc.)'
    )
    
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='raise',
        description='Controller type: raise, mpc, dreamerv3'
    )
    
    terrain_arg = DeclareLaunchArgument(
        'terrain',
        default_value='flat',
        description='Terrain type: flat, rough, stairs'
    )
    
    robot = LaunchConfiguration('robot')
    controller = LaunchConfiguration('controller')
    terrain = LaunchConfiguration('terrain')
    
    # RAISE controller node
    raise_controller_node = Node(
        package='locomotion',
        executable='raise_controller.py',
        name='raise_controller',
        parameters=[{
            'robot_model': robot,
            'terrain_type': terrain,
        }],
        condition=IfCondition(
            LaunchConfiguration('controller', default='raise')
        )
    )
    
    # MPC controller node
    mpc_controller_node = Node(
        package='locomotion',
        executable='mpc_node.py',
        name='mpc_controller',
        parameters=[{
            'robot_model': robot,
            'terrain_type': terrain,
            'horizon_length': 1.6,
        }],
        condition=IfCondition(
            LaunchConfiguration('controller', default='mpc')
        )
    )
    
    # Whole body controller node
    whole_body_controller = Node(
        package='locomotion',
        executable='whole_body_qp.py',
        name='whole_body_controller',
        parameters=[{
            'robot_model': robot,
            'control_frequency': 400,  # Hz
        }]
    )
    
    return LaunchDescription([
        robot_arg,
        controller_arg,
        terrain_arg,
        raise_controller_node,
        mpc_controller_node,
        whole_body_controller
    ])