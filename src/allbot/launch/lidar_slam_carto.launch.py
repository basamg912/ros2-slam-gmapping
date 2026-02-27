import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    # Directories
    allbot_dir = get_package_share_directory('allbot')
    allbot_urdf_dir = get_package_share_directory('allbot_urdf')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    
    # Launch Configurations
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                  default=os.path.join(allbot_dir, 'launch', 'include'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='allbot_lidar_standalone.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # YDLidar Params
    lidar_params_file = os.path.join(ydlidar_dir, 'params', 'X4.yaml')
    
    # Declare Arguments
    declare_cartographer_config_dir = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to the cartographer config directory to use'
    )
    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of the cartographer configuration file to use'
    )
    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value=resolution,
        description='Resolution of the map'
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value=publish_period_sec,
        description='OccupancyGrid publishing period'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )


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

    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[('scan', 'scan')]
    )

    # Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
    )

    # Shutdown handler: when cartographer_node exits, shutdown everything
    shutdown_on_cartographer_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=cartographer_node,
            on_exit=[Shutdown(reason='Cartographer node exited')]
        )
    )

    return LaunchDescription([
        declare_cartographer_config_dir,
        declare_configuration_basename,
        declare_resolution,
        declare_publish_period_sec,
        declare_use_sim_time,
        lidar_launch,
        urdf_launch,
        cartographer_node,
        occupancy_grid_node,
        shutdown_on_cartographer_exit,
    ])
