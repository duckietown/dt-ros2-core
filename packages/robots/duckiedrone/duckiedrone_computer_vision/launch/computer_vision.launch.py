# ruff: noqa: INP001
"""Launch composition for Duckiedrone computer vision."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the computer-vision stack."""
    args = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="",
            description="Robot namespace",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    optical_flow_dir = Path(get_package_share_directory("optical_flow"))

    optical_flow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            optical_flow_dir / "launch" / "optical_flow_node.launch.py",
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    return LaunchDescription(
        [
            *args,
            optical_flow_launch,
        ],
    )


if __name__ == "__main__":
    generate_launch_description()
