from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  turtle_name = DeclareLaunchArgument(
    'name',
    default_value="turtle1",
    description="Name of Turtle",
  )
  background_r = DeclareLaunchArgument(
    'bg_r',
    default_value="0",
    description="background color Red"
  )
  background_g = DeclareLaunchArgument(
    'bg_g',
    default_value="0",
    description="background color Green"
  )
  background_b = DeclareLaunchArgument(
    'bg_b',
    default_value="255",
    description="background color Blue"
  )
  spawn_turtle = ExecuteProcess(
    cmd=[
      'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
      '"{name: \'turtle2\', x: 2.0, y: 1.0, theta: 0.0}"'
    ],
    shell=True
  )
  turtle_node = Node(
    package="turtlesim",
    executable="turtlesim_node",
    name=LaunchConfiguration('name'),
    parameters=[{
      'background_r': LaunchConfiguration('bg_r'),
      'background_g': LaunchConfiguration('bg_g'),
      'background_b': LaunchConfiguration('bg_b'),
    }]
  )
  return LaunchDescription(
    [turtle_name, background_r, background_g, background_b, turtle_node, spawn_turtle]
  )