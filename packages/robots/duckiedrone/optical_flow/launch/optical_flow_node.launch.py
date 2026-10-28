# ruff: noqa: INP001
"""Launch file for the Duckiedrone optical flow node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the optical flow node."""
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

    pkg_dir = Path(get_package_share_directory("optical_flow"))
    config_file = str(pkg_dir / "config" / "optical_flow_node.yaml")

    node = Node(
        package="optical_flow",
        executable="optical_flow_node",
        name="optical_flow_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("~/image", "camera_node/image/compressed"),
            ("~/imu", "/mavros/imu/data"),
            ("~/altitude", "altitude_node/altitude"),
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
