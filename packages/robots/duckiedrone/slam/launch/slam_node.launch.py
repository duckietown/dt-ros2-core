# ruff: noqa: INP001
"""Launch file for the Duckiedrone SLAM node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the SLAM node launch description."""
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
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    node = Node(
        package="slam",
        executable="slam_node",
        name="slam_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("~/image", "camera_node/image/compressed"),
            ("~/state", "state_estimator_node/state"),
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
