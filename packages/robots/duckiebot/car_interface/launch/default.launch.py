from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    veh = LaunchConfiguration('veh')

    declare_veh = DeclareLaunchArgument(
        'veh', default_value='duckiebot', description='Vehicle name namespace'
    )
    
    # Get path to lane controller config file
    lane_control_share = get_package_share_directory('lane_control')
    lane_controller_config = os.path.join(lane_control_share, 'config', 'lane_controller.yaml')

    # Placeholder for ROS 2 executables; topic names/remaps mirror ROS 1 layout
    car_cmd_switch = Node(
        package='dagu_car',
        executable='car_cmd_switch_node',
        name='car_cmd_switch_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('cmd', 'car_cmd_switch_node/cmd'),
        ],
    )

    kinematics = Node(
        package='dagu_car',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('car_cmd', 'car_cmd_switch_node/cmd'),
            ('wheels_cmd', 'wheels_driver_node/wheels_cmd'),
            ('wheels_cmd_executed', 'wheels_driver_node/wheels_cmd_executed'),
            ('velocity', 'kinematics_node/velocity')
        ],
    )

    velocity_to_pose = Node(
        package='dagu_car',
        executable='velocity_to_pose_node',
        name='velocity_to_pose_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('velocity', 'kinematics_node/velocity'),
        ],
    )

    # Lane controller (ROS 2)
    lane_controller = Node(
        package='lane_control',
        executable='lane_controller',
        name='lane_controller_node',
        output='screen',
        parameters=[lane_controller_config],  # Load params from YAML file
        remappings=[
            ('car_cmd', 'lane_controller_node/car_cmd'),
            ('wheels_cmd_executed', 'wheels_driver_node/wheels_cmd_executed'),
            # Subscriptions below will be connected once upstream nodes are ported
            # ('lane_pose', 'lane_filter_node/lane_pose'),
            # ('stop_line_reading', 'stop_line_filter_node/stop_line_reading'),
            # ('wheels_cmd', 'wheels_cmd'),
            # ('fsm_mode', 'fsm_node/mode'),
        ],
    )

    joy_mapper = Node(
        package='joy_mapper',
        executable='joy_mapper',
        name='joy_mapper_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('car_cmd', 'joy_mapper_node/car_cmd'),
            ('emergency_stop', 'wheels_driver_node/emergency_stop'),
            ('joystick_override', 'joy_mapper_node/joystick_override'),
        ],
    )

    group = GroupAction([
        PushRosNamespace(veh),
        joy_mapper,
        kinematics,
        velocity_to_pose,
        lane_controller,
        car_cmd_switch,
    ])

    return LaunchDescription([
        declare_veh,
        group,
    ])
