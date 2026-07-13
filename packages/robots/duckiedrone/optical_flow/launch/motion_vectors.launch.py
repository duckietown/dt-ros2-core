#!/usr/bin/env python3
"""Launch the motion-vector preprocessing pipeline for Duckiedrone optical flow."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the motion-vector preprocessing graph."""
    args = [
        DeclareLaunchArgument("robot_name", default_value="", description="Robot namespace"),
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
    ]

    republish_node = Node(
        package="image_transport",
        executable="republish",
        name="republish_node",
        namespace=LaunchConfiguration("robot_name"),
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", LaunchConfiguration("input_image_topic")),
            ("out", LaunchConfiguration("output_image_topic")),
        ],
        output="screen",
    )

    resized_node = Node(
        package="image_proc",
        executable="crop_decimate_node",
        name="resized",
        namespace=LaunchConfiguration("robot_name"),
        remappings=[
            ("in/image_raw", LaunchConfiguration("output_image_topic")),
            ("in/camera_info", LaunchConfiguration("camera_info_topic")),
            ("out/image_raw", "camera_out/image_raw"),
            ("out/camera_info", "camera_out/camera_info"),
        ],
        parameters=[{"decimation_x": 8, "decimation_y": 8, "y_offset": 320, "height": 320, "width": 480}],
        output="screen",
    )

    flow_node = Node(
        package="opencv_apps",
        executable="fback_flow",
        name="fback_flow_node",
        namespace=LaunchConfiguration("robot_name"),
        remappings=[
            ("camera/camera_info", "camera_out/camera_info"),
            ("image", "camera_out/image_raw"),
        ],
        output="screen",
    )

    return LaunchDescription(args + [republish_node, resized_node, flow_node])


if __name__ == "__main__":
    generate_launch_description()