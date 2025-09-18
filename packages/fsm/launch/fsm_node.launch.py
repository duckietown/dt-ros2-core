#!/usr/bin/env python3
"""Launch file for the ROS 2 FSM node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml_parameters(package_name, config_path, param_file):
    """
    Load YAML parameters and convert nested structures to YAML strings.
    This is necessary because ROS 2 doesn't automatically flatten nested YAML structures.
    """
    pkg_share = get_package_share_directory(package_name)
    param_file_path = os.path.join(pkg_share, config_path, param_file)
    
    with open(param_file_path, 'r') as f:
        yaml_content = yaml.safe_load(f)
    
    # Extract parameters from the wildcard namespace
    raw_params = yaml_content['/**']['ros__parameters']
    
    # Convert nested parameters to YAML strings as expected by the node
    processed_params = {}
    for key, value in raw_params.items():
        if isinstance(value, (dict, list)):
            # Convert nested parameters to YAML strings
            processed_params[key] = yaml.dump(value, default_flow_style=False)
        else:
            # Keep simple parameters as-is
            processed_params[key] = value
    
    return [processed_params]


def generate_launch_description() -> LaunchDescription:
    veh = LaunchConfiguration('veh')
    param_file = LaunchConfiguration('param_file')

    # Load and process YAML parameters
    parameters = load_yaml_parameters('fsm', 'config/fsm_node', 'lane_following.yaml')
    print("PARAMETERS:", parameters)
    fsm_node = Node(
        package='fsm',
        executable='fsm_node',
        name='fsm_node',
        namespace=veh,
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'veh',
            description='Name of vehicle. ex: megaman',
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value='lane_following.yaml',
            description='Parameter file name',
        ),
        fsm_node,
    ])
