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
            name='joy_mapper_node',
            parameters=[config_file],
            output='screen',
            remappings=[
                ('car_cmd', 'joy_mapper_node/car_cmd'),
                ('emergency_stop', 'wheels_driver_node/emergency_stop'),
                ('joystick_override', 'joy_mapper_node/joystick_override'),
            ],
        ),
    ])
