
#!/usr/bin/env python3
"""Launch file for the ROS 2 lane filter node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('lane_filter')

    veh = LaunchConfiguration('veh')
    param_file = LaunchConfiguration('param_file')

    parameters = [
        PathJoinSubstitution([
            pkg_share,
            'config',
            'lane_filter_node',
            param_file,
        ])
    ]

    lane_filter = Node(
        package='lane_filter',
        executable='lane_filter_node',
        name='lane_filter_node',
        namespace=veh,
        output='screen',
        parameters=parameters,
    )

    return LaunchDescription([
        DeclareLaunchArgument('veh', default_value='', description='Vehicle namespace (e.g., megaman).'),
        DeclareLaunchArgument('param_file', default_value='default.yaml', description='Configuration file name.'),
        lane_filter,
    ])
