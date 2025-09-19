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
        package='dagu_car_ros2',
        executable='car_cmd_switch_node',
        name='car_cmd_switch_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('cmd', 'car_cmd'),
        ],
    )

    kinematics = Node(
        package='dagu_car_ros2',
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
        package='dagu_car_ros2',
        executable='velocity_to_pose_node',
        name='velocity_to_pose_node',
        output='screen',
        parameters=[{}],
        remappings=[
            ('velocity', 'kinematics/velocity'),
        ],
    )

    joy_mapper = Node(
        package='joy_mapper_ros2',
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
        car_cmd_switch,
    ])

    return LaunchDescription([
        declare_veh,
        group,
    ])
