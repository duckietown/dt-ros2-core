from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_control',
            executable='lane_controller_node',
            name='lane_controller_node',
            output='screen',
            remappings=[
                ('car_cmd', 'lane_controller_node/car_cmd'),
                ('wheels_cmd_executed', 'wheels_driver_node/wheels_cmd_executed'),
            ],
        ),
    ])
