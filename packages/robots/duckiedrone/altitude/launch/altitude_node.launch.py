#!/usr/bin/env python3
"""Launch file for the Duckiedrone altitude node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the altitude node."""

    args = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="",
            description="Robot namespace",
        ),
        DeclareLaunchArgument(
            "h_offset",
            default_value="0.02",
            description="Offset between ToF sensor origin and ground contact point.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal)",
        ),
    ]

    pkg_dir = get_package_share_directory("altitude")
    config_file = os.path.join(pkg_dir, "config", "altitude_node.yaml")

    altitude_node = Node(
        package="altitude",
        executable="altitude_node",
        name="altitude_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            config_file,
            {
                "h_offset": LaunchConfiguration("h_offset"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("~/imu", "/mavros/imu/data"),
            ("~/tof", "bottom_tof_driver_node/range"),
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(args + [altitude_node])


if __name__ == "__main__":
    generate_launch_description()