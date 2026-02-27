import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    allbot_urdf_dir = get_package_share_directory('allbot_urdf')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    map_frame = LaunchConfiguration('map_frame', default='map')
    
    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for the lidar scan')
        
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame of the robot')

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame')

    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame')

    # YDLidar Params
    lidar_params_file = os.path.join(ydlidar_dir, 'params', 'X4.yaml')

    # YDLidar Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={'params_file': lidar_params_file}.items()
    )

    # URDF Launch
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(allbot_urdf_dir, 'launch', 'urdf.launch.py')
        )
    )

    # Gmapping Node
    gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'map_update_interval': 5.0,
            'maxUrange': 10.0,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'lskip': 0,
            'srr': 0.1,
            'srt': 0.2,
            'str': 0.1,
            'stt': 0.2,
            'linearUpdate': 1.0,
            'angularUpdate': 0.5,
            'temporalUpdate': 3.0,
            'resampleThreshold': 0.5,
            'particles': 30,
            'xmin': -10.0,
            'ymin': -10.0,
            'xmax': 10.0,
            'ymax': 10.0,
            'delta': 0.05,
            'llsamplerange': 0.01,
            'llsamplestep': 0.01,
            'lasamplerange': 0.005,
            'lasamplestep': 0.005,
        }],
        remappings=[('scan', scan_topic)]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_scan_topic,
        declare_base_frame,
        declare_odom_frame,
        declare_map_frame,
        lidar_launch,
        urdf_launch,
        gmapping_node,
    ])
