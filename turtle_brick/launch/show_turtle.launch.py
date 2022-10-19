from distutils.command.config import LANG_EXT
from sympy import Parabola
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = get_package_share_path('turtle_brick')
    default_model_path = urdf_path / 'turtle.urdf.xacro'
    default_rviz_config_path = urdf_path / 'turtle_urdf.rviz'

    jsp_arg = DeclareLaunchArgument(name='j_s_p', default_value='gui', choices=['gui', 'jsp', 'none'],
                                    description='Flag to decide how to control joint_states')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='turtle_urdf', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=LaunchConfigurationEquals('j_s_p', 'jsp')
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('j_s_p', 'gui'),
    )

    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('turtle_urdf')],
    )

    return LaunchDescription([
        jsp_arg,
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
        ])