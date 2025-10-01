#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('joy_mapper')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'joy_mapper_node.yaml')

    return LaunchDescription([
        Node(
            package='joy_mapper',
            executable='joy_mapper',
            name='joy_mapper',
            parameters=[config_file],
            output='screen',
        ),
    ])
