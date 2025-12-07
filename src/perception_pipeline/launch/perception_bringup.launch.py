# launch/perception_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Isaac Sim) or real hardware'
    )
    
    use_sim = LaunchConfiguration('use_sim')
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_gyro': True,
            'enable_accel': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_sim', default='false'))
    )
    
    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam',
        parameters=[{
            'enable_rectified_pose': True,
            'enable_fisheye': False,
            'rectified_frame_id': 'camera_color_optical_frame',
            'map_frame_id': 'map',
            'publish_odom_tf': True,
        }],
        remappings=[
            ('/visual_slam/integrated_imu', '/camera/imu'),
            ('/visual_slam/camera_imu_optical_frame', '/camera/color/image_rect_raw'),
            ('/visual_slam/tracking/odometry', '/camera/odom/sample'),
        ]
    )
    
    # FoundationPose node for 6D object tracking
    foundationpose_node = Node(
        package='isaac_ros_foundationpose',
        executable='foundationpose_node',
        name='foundationpose',
        parameters=[{
            'input_image_width': 1280,
            'input_image_height': 720,
        }],
        remappings=[
            ('/image', '/camera/color/image_rect_raw'),
            ('/depth', '/camera/aligned_depth_to_color/image_raw'),
        ]
    )
    
    # OctoMap server for 3D mapping
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': 0.01,
            'frame_id': 'map',
            'sensor_model/max_range': 5.0,
        }],
        remappings=[
            ('/cloud_in', '/camera/depth/color/points'),
        ]
    )
    
    # People segmentation node
    people_segmentation_node = Node(
        package='isaac_ros_people_seg',
        executable='people_seg_node',
        name='people_segmentation',
        parameters=[{
            'input_width': 1280,
            'input_height': 720,
        }],
        remappings=[
            ('/image', '/camera/color/image_rect_raw'),
        ]
    )
    
    return LaunchDescription([
        use_sim_arg,
        realsense_node,
        visual_slam_node,
        foundationpose_node,
        octomap_server_node,
        people_segmentation_node
    ])