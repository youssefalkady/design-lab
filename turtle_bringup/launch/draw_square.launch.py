"""Start the whole system, then send one goal to the DrawSquare action."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    system_launch = os.path.join(
        get_package_share_directory('turtle_bringup'), 'launch', 'turtle_system.launch.py')
    side_length = LaunchConfiguration('side_length')

    return LaunchDescription([
        DeclareLaunchArgument(
            'side_length', default_value='3.0', description='Square side [m]'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='', description='See turtle_system'),
        DeclareLaunchArgument(
            'max_angular_speed', default_value='', description='See turtle_system'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            }.items()),

        # send_goal waits for the server itself; the delay just lets turtlesim open.
        TimerAction(period=2.0, actions=[ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '--feedback', '/draw_square',
                 'turtle_interfaces/action/DrawSquare',
                 ['{side_length: ', side_length, '}']],
            output='screen')]),
    ])
