"""Start turtlesim and every turtle_controller server with their parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _go_to_pose_node(context):
    # Speed args left empty keep the values from the YAML file.
    overrides = {}
    for name in ('max_linear_speed', 'max_angular_speed'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)
    params_file = LaunchConfiguration('params_file').perform(context)
    return [Node(
        package='turtle_controller', executable='go_to_pose_server',
        name='go_to_pose_server', output='screen',
        parameters=[params_file, overrides])]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('turtle_bringup'), 'config', 'turtle_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with the turtle_controller parameters'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='',
            description='Override max linear speed [m/s] (empty = use YAML)'),
        DeclareLaunchArgument(
            'max_angular_speed', default_value='',
            description='Override max angular speed [rad/s] (empty = use YAML)'),

        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        OpaqueFunction(function=_go_to_pose_node),
        Node(package='turtle_controller', executable='reset_server',
             name='reset_server', output='screen', parameters=[params_file]),
        Node(package='turtle_controller', executable='square_server',
             name='square_server', output='screen', parameters=[params_file]),
    ])
