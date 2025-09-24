#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(__file__), os.pardir)
    pkg_share = os.path.abspath(pkg_share)
    default_params = os.path.join(pkg_share, 'config', 'line_detector_node', 'default.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_params,
        description='Path to ROS 2 parameters YAML file'
    )

    node = Node(
        package='line_detector',
        executable='line_detector_node.py',
        name='line_detector_node',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription([
        param_file_arg,
        node,
    ])

