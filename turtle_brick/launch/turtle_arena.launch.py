import os
from struct import pack

from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    run_turtle_launch_file = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtle_brick')), '/run_turtle.launch.py']))

    arena_node = Node(package='turtle_brick',
        executable='arena'
    )
    
    catcher_node = Node(package='turtle_brick',
        executable='catcher'
    )

    return LaunchDescription([
        run_turtle_launch_file,
        arena_node,
        catcher_node
    ])