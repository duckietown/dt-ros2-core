#!/usr/bin/env python3
"""Launch file for the Duckiedrone fly commands mux node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the fly commands mux node."""
    args = [
        DeclareLaunchArgument("robot_name", default_value="", description="Robot namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal)",
        ),
    ]

    pkg_dir = get_package_share_directory("fly_commands_mux")
    config_file = os.path.join(pkg_dir, "config", "fly_commands_mux_node.yaml")

    node = Node(
        package="fly_commands_mux",
        executable="fly_commands_mux_node",
        name="fly_commands_mux_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("~/commands/output", "flight_controller_node/commands")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(args + [node])


if __name__ == "__main__":
    generate_launch_description()