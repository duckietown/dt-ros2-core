from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    veh = LaunchConfiguration("veh")

    return LaunchDescription([
        DeclareLaunchArgument(
            "veh",
            default_value="",
            description="Vehicle namespace (e.g., megaman)",
        ),
        GroupAction([
            PushRosNamespace(veh),
            Node(
                package="ground_projection",
                executable="ground_projection_node",
                name="ground_projection_node",
                output="screen",
                remappings=[
                    ("camera_info", "camera_node/camera_info"),
                ],
            ),
        ]),
    ])

