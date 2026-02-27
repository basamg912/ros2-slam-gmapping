from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = []
    
    # Declare the 'gui' argument (default false to match plan)
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            "xacro",
            " ",
            PathJoinSubstitution(
                [FindPackageShare("allbot_urdf"), "urdf", "allbot.urdf.xacro"]
            ),
        ]
    )
    # Wrap in ParameterValue to prevent YAML parsing errors
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint State Publisher (Standard)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=None, # Run always for now, or use 'unless gui' logic if separate gui node exists
    )

    return LaunchDescription(
        declared_arguments + 
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
