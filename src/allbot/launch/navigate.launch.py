import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    allbot_param_dir = get_package_share_directory('allbot_param')
    allbot_dir = get_package_share_directory('allbot')
    allbot_urdf_dir = get_package_share_directory('allbot_urdf')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    launch_urdf = LaunchConfiguration('launch_urdf')
    launch_lidar = LaunchConfiguration('launch_lidar')
    
    # Default paths (used in DeclareLaunchArgument)
    default_params_file = os.path.join(allbot_param_dir, 'navigation', 'nav2_params.yaml')
    default_map_file = os.path.join(allbot_dir, 'maps', 'my_map.yaml')
    
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(allbot_urdf_dir, 'launch', 'urdf.launch.py')
        ),
        condition=IfCondition(launch_urdf)
    )
    lidar_params_file = os.path.join(ydlidar_dir, 'params', 'X4.yaml')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={'params_file': lidar_params_file}.items(),
        condition=IfCondition(launch_lidar)
    )
    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Set to True if running SLAM simultaneously (Gmapping provides map frame). '
                    'Set to False to navigate on a saved map (AMCL provides localization).')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file (only used when slam:=False)')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_launch_urdf_cmd = DeclareLaunchArgument(
        'launch_urdf',
        default_value='true',
        description='Whether to launch the URDF (robot_state_publisher). Set to false if running bringup.')

    declare_launch_lidar_cmd = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Whether to launch the Lidar. Set to false if already running.')

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(slam)
    )
    # Localization: map_server + AMCL (only in saved-map mode)
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam)
    )

    # Navigation: controller/planner/behavior servers (only in saved-map mode)
    nav2_navigation_saved_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
        condition=UnlessCondition(slam)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_launch_urdf_cmd,
        declare_launch_lidar_cmd,
        lidar_launch,
        nav2_navigation_launch,
        urdf_launch,
        nav2_localization_launch,
        nav2_navigation_saved_launch,
    ])
