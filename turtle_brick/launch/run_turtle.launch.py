import os

from struct import pack
from termios import ECHOE
from tkinter.messagebox import NO
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

    turtlesim_node = Node(package='turtlesim', executable='turtlesim_node')

    turtle_robot_node = Node(package='turtle_brick',
        executable='turtle_robot'
    )


    return LaunchDescription([
        turtlesim_node,
        turtle_robot_node,
        show_turtle_launch_file
    ])