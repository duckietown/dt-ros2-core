# ruff: noqa: INP001
"""Launch file for the Duckiedrone height and velocity controller."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the controller node."""
    args = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="",
            description="Robot namespace",
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

    pkg_dir = Path(get_package_share_directory("pid_controller"))
    virtual = os.environ.get("ROBOT_HARDWARE") == "virtual"
    hardware = "virtual" if virtual else "physical"
    config_file = str(
        pkg_dir / "config" / f"pid_controller_node_{hardware}.yaml",
    )

    node = Node(
        package="pid_controller",
        executable="pid_controller_node",
        name="pid_controller_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("~/fcu_state", "/mavros/state"),
            ("~/imu", "/mavros/imu/data"),
            ("~/battery", "/mavros/battery"),
            ("~/altitude", "altitude_node/altitude"),
            ("~/velocity", "optical_flow_node/velocity"),
            ("~/flow_heartbeat", "optical_flow_node/heartbeat"),
            ("~/link", "/live_tap/link"),
            ("~/attitude", "/mavros/setpoint_attitude/attitude"),
            ("~/thrust", "/mavros/setpoint_attitude/thrust"),
            ("~/manual_control", "/mavros/manual_control/send"),
            ("~/set_mode", "/mavros/set_mode"),
            ("~/arming", "/mavros/cmd/arming"),
            ("~/command", "/mavros/cmd/command"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        output="screen",
        emulate_tty=True,
        sigterm_timeout="15",
    )

    return LaunchDescription([*args, node])


if __name__ == "__main__":
    generate_launch_description()
