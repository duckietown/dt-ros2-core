
#!/usr/bin/env python3
"""Launch file for the ROS 2 lane filter tester node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    veh = LaunchConfiguration('veh')

    tester_params = {
        'x1': LaunchConfiguration('x1'),
        'y1': LaunchConfiguration('y1'),
        'x2': LaunchConfiguration('x2'),
        'y2': LaunchConfiguration('y2'),
        'color': LaunchConfiguration('color'),
    }

    tester_node = Node(
        package='lane_filter',
        executable='lane_filter_tester_node',
        name='lane_filter_tester_node',
        namespace=veh,
        output='screen',
        parameters=[tester_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('veh', default_value='', description='Vehicle namespace (e.g., megaman).'),
        DeclareLaunchArgument('x1', default_value='0.0', description='First point x coordinate.'),
        DeclareLaunchArgument('y1', default_value='0.0', description='First point y coordinate.'),
        DeclareLaunchArgument('x2', default_value='0.0', description='Second point x coordinate.'),
        DeclareLaunchArgument('y2', default_value='0.0', description='Second point y coordinate.'),
        DeclareLaunchArgument('color', default_value='white', description='Segment color (white/yellow/red).'),
        tester_node,
    ])
