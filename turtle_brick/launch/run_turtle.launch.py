import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    show_turtle_launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtle_brick')), '/show_turtle.launch.py']))

    config = os.path.join(
      get_package_share_directory('turtle_brick'),
      'turtle.yaml'
      )

    turtle_robot_node = Node(package='turtle_brick',
        executable='turtle_robot',
        parameters=[config]
    )

    turtlesim_node = Node(package='turtlesim', executable='turtlesim_node')


    return LaunchDescription([
        show_turtle_launch_file,
        turtle_robot_node,
        turtlesim_node
    ])