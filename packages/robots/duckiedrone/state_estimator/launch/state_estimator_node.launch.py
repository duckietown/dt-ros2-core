# ruff: noqa: INP001
"""Launch file for the Duckiedrone EMA state estimator node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the EMA state estimator node."""
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

    pkg_dir = Path(get_package_share_directory("state_estimator"))
    config_file = str(pkg_dir / "config" / "state_estimator_node.yaml")

    node = Node(
        package="state_estimator",
        executable="state_estimator_node",
        name="state_estimator_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("~/imu", "/mavros/imu/data"),
            ("~/twist", "optical_flow_node/debug/raw_odometry"),
            ("~/range", "bottom_tof_driver_node/range"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([*args, node])


if __name__ == "__main__":
    generate_launch_description()
