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
        DeclareLaunchArgument(
            "input_image_topic",
            default_value="camera_node/image/compressed",
            description="Compressed image topic to decode for flow.",
        ),
        DeclareLaunchArgument(
            "output_image_topic",
            default_value="camera_node/image_raw",
            description="Decoded raw image topic.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="camera_node/camera_info",
            description="Camera info topic for the optical-flow pipeline.",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    optical_flow_dir = Path(get_package_share_directory("optical_flow"))
    ground_projection_dir = Path(
        get_package_share_directory("ground_projection"),
    )

    motion_vectors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            optical_flow_dir / "launch" / "motion_vectors.launch.py",
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "input_image_topic": LaunchConfiguration("input_image_topic"),
            "output_image_topic": LaunchConfiguration("output_image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        }.items(),
    )

    optical_flow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            optical_flow_dir / "launch" / "optical_flow_node.launch.py",
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    ground_projection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ground_projection_dir
            / "launch"
            / "ground_projection.launch.py",
        ),
        launch_arguments={
            "veh": LaunchConfiguration("robot_name"),
        }.items(),
    )

    return LaunchDescription(
        [
            *args,
            motion_vectors_launch,
            optical_flow_launch,
            ground_projection_launch,
        ],
    )


if __name__ == "__main__":
    generate_launch_description()
