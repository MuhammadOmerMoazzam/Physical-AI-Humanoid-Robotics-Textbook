# src/vla/launch/voice_to_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='rdt1b-1.2b-8bit',
        description='VLA model to use: rdt1b-1.2b-8bit, openvla-7b, octo-small'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run the model on: cuda, cpu'
    )
    
    model = LaunchConfiguration('model')
    device = LaunchConfiguration('device')
    
    # Whisper voice recognition node
    whisper_node = Node(
        package='vla',
        executable='whisper_node.py',
        name='whisper_voice_recognition',
        parameters=[{
            'model_size': 'large-v3',
            'language': 'en',
            'device': device,
        }],
        remappings=[
            ('/microphone/audio_input', '/audio/input'),
            ('/speech_to_text', '/natural_language_command'),
        ]
    )
    
    # VLA model node (select based on model arg)
    vla_node = Node(
        package='vla',
        executable='vla_node_selector.py',  # This would handle model selection
        name='vla_controller',
        parameters=[{
            'model_name': model,
            'device': device,
            'control_frequency': 100,  # Hz
        }],
        remappings=[
            ('/camera/color/image_raw', '/humanoid/camera_front/image_raw'),
            ('/natural_language_command', '/natural_language_command'),
            ('/joint_commands', '/humanoid/joint_group_position_controller/commands'),
        ]
    )
    
    # Action chunking node to convert VLA outputs to smooth control
    action_chunker = Node(
        package='vla',
        executable='action_chunker_node.py',
        name='action_chunker',
        parameters=[{
            'chunk_size': 50,  # RDT-1B outputs 50 actions per forward pass
            'control_frequency': 100,
            'interpolation_method': 'linear',
        }]
    )
    
    # Feedback and visualization node
    feedback_node = Node(
        package='vla',
        executable='feedback_visualizer.py',
        name='vla_feedback_visualizer',
        parameters=[{
            'display_predictions': True,
            'publish_markers': True,
        }],
        condition=IfCondition(LaunchConfiguration('visualize', default='true')),
        remappings=[
            ('/vla/debug_text', '/vla_debug_text'),
        ]
    )
    
    return LaunchDescription([
        model_arg,
        device_arg,
        whisper_node,
        vla_node,
        action_chunker,
        feedback_node
    ])