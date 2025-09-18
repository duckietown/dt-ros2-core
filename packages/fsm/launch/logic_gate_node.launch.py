#!/usr/bin/env python3
"""Launch file for the ROS 2 Logic Gate node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('fsm')

    veh = LaunchConfiguration('veh')
    param_file = LaunchConfiguration('param_file')

    parameters = [
        PathJoinSubstitution([
            pkg_share,
            'config',
            'logic_gate_node',
            param_file,
        ])
    ]

    logic_gate_node = Node(
        package='fsm',
        executable='logic_gate_node',
        name='logic_gate_node',
        namespace=veh,
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'veh',
            description='Name of vehicle. ex: megaman',
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value='default.yaml',
            description='Parameter file name',
        ),
        logic_gate_node,
    ])
