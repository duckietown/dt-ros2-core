from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    veh = LaunchConfiguration('veh')
    
    pkg_share = get_package_share_directory('led_emitter')
    config_path = os.path.join(pkg_share, 'config', 'led_emitter_node', 'LED_protocol.yaml')
    params = {
        'protocol_file': config_path,
        'LED_scale': 0.8,
        'robot_type': 'duckiebot',
    }

    node = Node(
        package='led_emitter',
        executable='led_emitter_node',
        name='led_emitter_node',
        namespace=veh,
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'veh',
            default_value=os.getenv('VEHICLE_NAME'),
            description='Name of vehicle/robot (used as namespace)',
        ),
        node,
    ])


