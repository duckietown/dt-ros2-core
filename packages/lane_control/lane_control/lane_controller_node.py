import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, StopLineReading


class LaneControllerNode(Node):
    def __init__(self):
        # Use a clean node name without the `_node` suffix
        super().__init__('lane_controller')

        # Parameters (typed, with defaults). These can be overridden via params file.
        self.declare_parameter('v_bar', 0.25)
        self.declare_parameter('k_d', 3.0)
        self.declare_parameter('k_theta', 8.0)
        self.declare_parameter('d_thres', 0.3)
        self.declare_parameter('theta_thres_min', -0.6)
        self.declare_parameter('theta_thres_max', 0.6)
        self.declare_parameter('d_offset', 0.0)
        self.declare_parameter('omega_ff', 0.0)

        # Internal state
        self._last_time_sec: Optional[float] = None
        self._wheels_cmd_executed = WheelsCmdStamped()
        self._stop_line_distance: Optional[float] = None
        self._stop_line_detected: bool = False
        self._at_stop_line: bool = False
        self._obstacle_stop_line_distance: Optional[float] = None
        self._obstacle_stop_line_detected: bool = False
        self._at_obstacle_stop_line: bool = False
        self._current_pose_source: str = 'lane_filter'

        # QoS profiles
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = HistoryPolicy.KEEP_LAST

        qos_cmd = QoSProfile(depth=1)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.history = HistoryPolicy.KEEP_LAST

        # Publisher
        self.pub_car_cmd = self.create_publisher(Twist2DStamped, 'car_cmd', qos_cmd)

        # Subscribers (relative topic names; namespace comes from ROS_NAMESPACE or launch)
        self.create_subscription(LanePose, 'lane_pose', self._cb_lane_pose, qos_sensor)
        self.create_subscription(LanePose, 'intersection_navigation_pose', self._cb_intersection_pose, qos_sensor)
        self.create_subscription(WheelsCmdStamped, 'wheels_cmd_executed', self._cb_wheels_executed, qos_sensor)
        self.create_subscription(StopLineReading, 'stop_line_reading', self._cb_stop_line, qos_sensor)
        self.create_subscription(StopLineReading, 'obstacle_distance_reading', self._cb_obstacle_stop_line, qos_sensor)

        self.get_logger().info('lane_controller initialized (ROS 2)')

    # --- Subscribers -----------------------------------------------------
    def _cb_wheels_executed(self, msg: WheelsCmdStamped):
        self._wheels_cmd_executed = msg

    def _cb_stop_line(self, msg: StopLineReading):
        self._stop_line_distance = -msg.stop_pose.x
        self._stop_line_detected = msg.stop_line_detected
        self._at_stop_line = msg.at_stop_line
        if not self._stop_line_detected:
            self._stop_line_distance = None

    def _cb_obstacle_stop_line(self, msg: StopLineReading):
        self._obstacle_stop_line_distance = math.hypot(msg.stop_pose.x, msg.stop_pose.y)
        self._obstacle_stop_line_detected = msg.stop_line_detected
        self._at_obstacle_stop_line = msg.at_stop_line
        if not self._obstacle_stop_line_detected:
            self._obstacle_stop_line_distance = None

    def _cb_lane_pose(self, msg: LanePose):
        self._handle_pose(msg, source='lane_filter')

    def _cb_intersection_pose(self, msg: LanePose):
        self._handle_pose(msg, source='intersection_navigation')

    # --- Core logic ------------------------------------------------------
    def _handle_pose(self, pose: LanePose, source: str):
        if source != self._current_pose_source:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        dt = None if self._last_time_sec is None else max(0.0, now_sec - self._last_time_sec)
        self._last_time_sec = now_sec

        # Read parameters
        v_bar = float(self.get_parameter('v_bar').value)
        k_d = float(self.get_parameter('k_d').value)
        k_theta = float(self.get_parameter('k_theta').value)
        d_thres = float(self.get_parameter('d_thres').value)
        theta_min = float(self.get_parameter('theta_thres_min').value)
        theta_max = float(self.get_parameter('theta_thres_max').value)
        d_offset = float(self.get_parameter('d_offset').value)
        omega_ff = float(self.get_parameter('omega_ff').value)

        # Safety: stop at stop lines
        if self._at_stop_line or self._at_obstacle_stop_line:
            v = 0.0
            omega = 0.0
        else:
            # Errors
            d_err = pose.d - d_offset
            if abs(d_err) > d_thres:
                d_err = math.copysign(d_thres, d_err)

            phi_err = pose.phi
            phi_err = min(max(phi_err, theta_min), theta_max)

            # Minimal P-control (placeholder for full controller)
            v = v_bar
            omega = -k_d * d_err - k_theta * phi_err + omega_ff

        # Publish command
        cmd = Twist2DStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = pose.header.frame_id
        cmd.v = float(v)
        cmd.omega = float(omega)
        self.pub_car_cmd.publish(cmd)


def main():
    rclpy.init()
    node = LaneControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
