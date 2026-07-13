#!/usr/bin/env python3
"""Launch wrapper for the default Duckiedrone interface stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the default Duckiedrone interface stack."""
    args = [
        DeclareLaunchArgument("robot_name", default_value="", description="Robot namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal)",
        ),
    ]

    fly_commands_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fly_commands_mux"),
                "launch",
                "fly_commands_mux_node.launch.py",
            )
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    return LaunchDescription(args + [fly_commands_mux_launch])


if __name__ == "__main__":
    generate_launch_description()