from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_control',
            executable='lane_controller_node',
            name='lane_controller',
            output='screen',
        ),
    ])
