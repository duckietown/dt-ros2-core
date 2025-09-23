from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    veh = LaunchConfiguration('veh')

    declare_veh = DeclareLaunchArgument(
        'veh', default_value='duckiebot', description='Vehicle name namespace'
    )

    # Placeholder for ROS 2 executables; topic names/remaps mirror ROS 1 layout
    car_cmd_switch = Node(
        package='dagu_car',
        executable='car_cmd_switch_node',
        name='car_cmd_switch_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('cmd', 'car_cmd'),
        ],
    )

    kinematics = Node(
        package='dagu_car',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('car_cmd', 'car_cmd'),
            ('wheels_cmd', 'wheels_cmd'),
            ('wheels_cmd_executed', 'wheels_cmd_executed'),
            ('velocity', 'kinematics/velocity')
        ],
    )

    velocity_to_pose = Node(
        package='dagu_car',
        executable='velocity_to_pose_node',
        name='velocity_to_pose_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('velocity', 'kinematics/velocity'),
        ],
    )

    # Lane controller (ROS 2)
    lane_controller = Node(
        package='lane_control',
        executable='lane_controller_node',
        name='lane_controller_node',
        output='screen',
        # Keep defaults; perception pipeline not yet wired in ROS2
        parameters=[{}],
        remappings=[
            # Publish car_cmd under the expected source for car_cmd_switch
            ('car_cmd', 'lane_controller_node/car_cmd'),
            # Subscriptions below will be connected once upstream nodes are ported
            # ('lane_pose', 'lane_filter_node/lane_pose'),
            # ('stop_line_reading', 'stop_line_filter_node/stop_line_reading'),
            # ('wheels_cmd', 'wheels_cmd'),
            # ('fsm_mode', 'fsm_node/mode'),
        ],
    )

    joy_mapper = Node(
        package='joy_mapper',
        executable='joy_mapper_node',
        name='joy_mapper',
        output='screen',
        parameters=[{}],
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
