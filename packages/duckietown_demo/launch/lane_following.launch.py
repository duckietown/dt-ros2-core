#!/usr/bin/env python3
"""
Lane Following Demo Launch File

Launches the full lane-following stack for a Duckiebot:
  - line_detector_node    : detects lane markings in the camera image
  - ground_projection_node: projects detected segments onto the ground plane
  - lane_filter_node      : estimates the robot's pose within the lane
  - lane_controller_node  : computes wheel commands to keep the robot in the lane
  - fsm_node              : finite state machine that orchestrates the demo
  - led_emitter_node      : controls the robot's LEDs based on FSM state

Mirrors the ROS 1 lane_following.launch from dt-core/ente with the nodes
enabled in that file:
    fsm=true, lane_following=true (line_detection, ground_projection,
    lane_filter, lane_controller), LED=true (/LED/emitter=true)

Usage:
    ros2 launch duckietown_demo lane_following.launch.py veh:=<robot_name>
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_yaml_parameters(package_name: str, config_path: str, param_file: str) -> list:
    """Load a ROS 2 YAML parameter file and return a list suitable for Node(parameters=…).

    Nested dicts/lists are serialised back to YAML strings so that the node
    can parse them without additional ROS 2 flattening magic.
    """
    pkg_share = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_share, config_path, param_file)

    with open(full_path, "r") as fh:
        raw = yaml.safe_load(fh)

    # Support both '/**' and direct 'ros__parameters' layouts
    if "/**" in raw:
        ros_params = raw["/**"]["ros__parameters"]
    elif "ros__parameters" in raw:
        ros_params = raw["ros__parameters"]
    else:
        ros_params = raw

    processed: dict = {}
    for key, value in ros_params.items():
        if isinstance(value, (dict, list)):
            processed[key] = yaml.dump(value, default_flow_style=False)
        else:
            processed[key] = value

    return [processed]


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description() -> LaunchDescription:
    veh = LaunchConfiguration("veh")
    param_file = LaunchConfiguration("param_file")

    # -----------------------------------------------------------------------
    # Package share directories
    # -----------------------------------------------------------------------
    line_detector_share = get_package_share_directory("line_detector")
    ground_projection_share = get_package_share_directory("ground_projection")
    lane_filter_share = get_package_share_directory("lane_filter")
    led_emitter_share = get_package_share_directory("led_emitter")

    # -----------------------------------------------------------------------
    # Node: line_detector_node
    #   Detects coloured line segments in the rectified camera image.
    #   Subscribes : ~image/compressed  (camera image)
    #   Publishes  : ~segment_list      (detected segments)
    # -----------------------------------------------------------------------
    line_detector_params = PathJoinSubstitution([
        line_detector_share,
        "config",
        "line_detector_node",
        param_file,
    ])

    line_detector_node = Node(
        package="line_detector",
        executable="line_detector_node.py",
        name="line_detector_node",
        namespace=veh,
        output="screen",
        parameters=[line_detector_params],
        remappings=[
            # Camera image comes from the camera node
            ("image/compressed", "camera_node/image/compressed"),
        ],
    )

    # -----------------------------------------------------------------------
    # Node: ground_projection_node
    #   Projects image-space segments onto the ground plane using the
    #   homography calibration.
    #   Subscribes : ~lineseglist_in    (from line_detector)
    #                ~camera_info       (camera intrinsics)
    #   Publishes  : ~lineseglist_out   (ground-projected segments)
    # -----------------------------------------------------------------------
    ground_projection_params_file = os.path.join(
        ground_projection_share, "config", "ground_projection_node", "default.yaml"
    )
    ground_projection_params = [ground_projection_params_file] \
        if os.path.isfile(ground_projection_params_file) else []

    ground_projection_node = Node(
        package="ground_projection",
        executable="ground_projection_node",
        name="ground_projection_node",
        namespace=veh,
        output="screen",
        parameters=ground_projection_params,
        remappings=[
            # Receive segments from the line detector
            ("lineseglist_in", "line_detector_node/segment_list"),
            # Camera info from the camera node
            ("camera_info", "camera_node/camera_info"),
        ],
    )

    # -----------------------------------------------------------------------
    # Node: lane_filter_node
    #   Estimates the robot's lateral offset (d) and heading error (phi)
    #   within the lane from ground-projected segments.
    #   Subscribes : ~segment_list      (from ground_projection)
    #                ~car_cmd           (from lane_controller, for prediction)
    #   Publishes  : ~lane_pose         (d, phi estimate)
    # -----------------------------------------------------------------------
    lane_filter_params = PathJoinSubstitution([
        lane_filter_share,
        "config",
        "lane_filter_node",
        param_file,
    ])

    lane_filter_node = Node(
        package="lane_filter",
        executable="lane_filter_node",
        name="lane_filter_node",
        namespace=veh,
        output="screen",
        parameters=[lane_filter_params],
        remappings=[
            # Receive ground-projected segments
            ("segment_list", "ground_projection_node/lineseglist_out"),
            # Receive car commands for motion prediction
            ("car_cmd", "lane_controller_node/car_cmd"),
        ],
    )

    # -----------------------------------------------------------------------
    # Node: lane_controller_node
    #   Computes wheel velocity commands to minimise d and phi.
    #   Subscribes : ~lane_pose         (from lane_filter)
    #   Publishes  : ~car_cmd           (wheel commands)
    # -----------------------------------------------------------------------
    lane_controller_node = Node(
        package="lane_control",
        executable="lane_controller_node",
        name="lane_controller_node",
        namespace=veh,
        output="screen",
        remappings=[
            # Receive lane pose from lane filter
            ("lane_pose", "lane_filter_node/lane_pose"),
        ],
    )

    # -----------------------------------------------------------------------
    # Node: fsm_node
    #   Finite State Machine that orchestrates the demo mode transitions
    #   (e.g. LANE_FOLLOWING ↔ JOYSTICK_CONTROL).
    #   Uses the lane_following FSM configuration.
    # -----------------------------------------------------------------------
    fsm_parameters = _load_yaml_parameters("fsm", "config/fsm_node", "lane_following.yaml")

    fsm_node = Node(
        package="fsm",
        executable="fsm_node",
        name="fsm_node",
        namespace=veh,
        output="screen",
        parameters=fsm_parameters,
        remappings=[
            # Forward FSM set_pattern requests to the LED emitter
            ("fsm_node/set_pattern", "set_pattern"),
            # Receive joystick mode-override commands
            ("mode_override", "joy_mapper_node/mode_override"),
            # Receive car commands from the lane controller
            ("car_cmd_in", "lane_controller_node/car_cmd"),
        ],
    )

    # -----------------------------------------------------------------------
    # Node: led_emitter_node
    #   Controls the RGB LEDs on the Duckiebot based on the FSM state /
    #   explicit set_pattern requests.
    # -----------------------------------------------------------------------
    led_protocol_file = os.path.join(
        led_emitter_share, "config", "led_emitter_node", "LED_protocol.yaml"
    )
    led_emitter_params = {
        "protocol_file": led_protocol_file,
        "LED_scale": 0.8,
        "robot_type": "duckiebot",
    }

    led_emitter_node = Node(
        package="led_emitter",
        executable="led_emitter_node",
        name="led_emitter_node",
        namespace=veh,
        output="screen",
        parameters=[led_emitter_params],
    )

    # -----------------------------------------------------------------------
    # Assemble launch description
    # -----------------------------------------------------------------------
    return LaunchDescription([
        # ---- launch arguments ----
        DeclareLaunchArgument(
            "veh",
            default_value=os.getenv("VEHICLE_NAME", ""),
            description="Vehicle (robot) name used as the ROS namespace, e.g. megaman",
        ),
        DeclareLaunchArgument(
            "param_file",
            default_value="default.yaml",
            description="Parameter file name (without path) used by line_detector and lane_filter nodes",
        ),

        # ---- all nodes share the vehicle namespace ----
        GroupAction([
            PushRosNamespace(veh),

            # Vision pipeline: camera → line detection → ground projection → lane filter
            line_detector_node,
            ground_projection_node,
            lane_filter_node,

            # Control: lane controller produces wheel commands
            lane_controller_node,

            # Orchestration: FSM manages demo state transitions
            fsm_node,

            # Actuation: LED emitter reflects FSM state
            led_emitter_node,
        ]),
    ])
