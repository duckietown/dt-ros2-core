#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os


def generate_launch_description():
    veh = LaunchConfiguration('veh')

    # Optional: use line_detector default params if available
    ld_pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, 'line_detector'))
    ld_default_params = os.path.join(ld_pkg_share, 'config', 'line_detector_node', 'default.yaml')
    params = [ld_default_params] if os.path.isfile(ld_default_params) else []

    return LaunchDescription([
        DeclareLaunchArgument(
            'veh',
            default_value='',
            description='Vehicle namespace (e.g., megaman)'
        ),
        GroupAction([
            PushRosNamespace(veh),

            # Line detector node
            Node(
                package='line_detector',
                executable='line_detector_node.py',
                name='line_detector_node',
                output='screen',
                parameters=params,
                # Common Duckietown camera naming is camera_node/image/compressed; if needed, remap here.
                # remappings=[('image/compressed', 'camera_node/image/compressed')],
            ),

            # Ground projection node
            Node(
                package='ground_projection',
                executable='ground_projection_node',
                name='ground_projection_node',
                output='screen',
                # Remap the GP subscriber to the line detector output
                remappings=[
                    ('lineseglist_in', 'segment_list'),
                    # If your camera_info comes from camera_node, enable the next line
                    # ('camera_info', 'camera_node/camera_info'),
                ],
            ),
        ]),
    ])

