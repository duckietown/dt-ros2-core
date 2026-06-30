#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os


def generate_launch_description():
    veh = LaunchConfiguration('veh')
    pkg_share = os.path.join(os.path.dirname(__file__), os.pardir)
    pkg_share = os.path.abspath(pkg_share)
    default_params = os.path.join(pkg_share, 'config', 'line_detector_node', 'default.yaml')

    veh_arg = DeclareLaunchArgument(
        'veh',
        default_value='',
        description='Vehicle namespace (e.g., megaman)'
    )

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_params,
        description='Path to ROS 2 parameters YAML file'
    )

    return LaunchDescription([
        veh_arg,
        param_file_arg,
        GroupAction([
            PushRosNamespace(veh),
            Node(
                package='line_detector',
                executable='line_detector_node.py',
                name='line_detector_node',
                output='screen',
                parameters=[LaunchConfiguration('param_file')],
                remappings=[
                    ('image/compressed', 'camera_node/image/compressed'),
                ],
            ),
        ]),
    ])

