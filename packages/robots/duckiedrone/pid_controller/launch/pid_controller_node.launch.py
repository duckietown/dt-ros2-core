# ruff: noqa: INP001
"""Launch file for the Duckiedrone PID controller node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the PID controller node."""
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
    config_file = str(pkg_dir / "config" / "pid_controller_node.yaml")

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
            ("~/mode", "flight_controller_node/mode/current"),
            ("~/commands", "mavros/setpoint_raw/attitude"),
            ("~/state", "state_estimator_node/state"),
            ("~/set_mode", "flight_controller_node/set_mode"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([*args, node])


if __name__ == "__main__":
    generate_launch_description()
