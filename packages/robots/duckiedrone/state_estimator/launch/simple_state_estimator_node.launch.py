#!/usr/bin/env python3
"""Launch file for the Duckiedrone simple state estimator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the simple state estimator node."""
    args = [
        DeclareLaunchArgument("robot_name", default_value="", description="Robot namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal)",
        ),
    ]

    node = Node(
        package="state_estimator",
        executable="simple_state_estimator_node",
        name="state_estimator_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("~/altitude", "altitude_node/altitude")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(args + [node])


if __name__ == "__main__":
    generate_launch_description()