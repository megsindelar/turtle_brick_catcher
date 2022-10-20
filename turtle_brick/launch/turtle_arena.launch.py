import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    run_turtle_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('turtle_brick')), '/run_turtle.launch.py']))

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
