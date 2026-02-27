from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    allbot_param_path = FindPackageShare('allbot_param')
    
    # EKF Config
    ekf_config_path = PathJoinSubstitution(
        [allbot_param_path, 'ekf', 'robot_localization.yaml']
    )

    return LaunchDescription([
        # Allbot Driver (replaces rosserial)
        Node(
            package="serial_bridge",
            executable="serial_bridge",
            name="serial_bridge",
            output="screen",
        ),
        Node(
            package='allbot_base',
            executable='allbot_base_node',
            name='allbot_base_node',
            output='screen',
            parameters=[
                {'linear_scale': 1.0},
                {'serial_port': '/dev/ttyTHS1'},
                {'baud_rate': 115200}
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('odometry/filtered', 'odom')]
        ),
        
        # MPU6050
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros2_mpu6050'),
                    'launch',
                    'ros2_mpu6050.launch.py'
                ])
            )
        ),
        # IMU complementary Filter
        Node(
            package="imu_complementary_filter",
            executable="complementary_filter_node",
            name="imu_filter_node_for_orientation",
            parameters=[
                {"fixed_frame" : "base_footprint"},
                {"use_mag" : False},
                {"use_magnetic_field_msg" : False},
            ]
        ),
        
        # URDF and Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('allbot_urdf'),
                    'launch',
                    'urdf.launch.py'
                ])
            )
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_imu_link",
            arguments=[
                "0", "0", "0", "0", "0", "0", "base_footprint", "imu_link"
            ]
        )
    ])
