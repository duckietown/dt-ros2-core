"""Lane controller node (ROS 2).

Port of ``lane_controller_node.py`` from ``dt-core`` (ROS 1): computes car commands (v, omega) from
the lane pose using the PI(D) ``LaneController`` with anti-windup, error dead-bands and stop-line
slowdown. Parameters and their defaults are the same as in ROS 1 (see
``config/lane_controller_node/default.yaml``).

Subscribes (relative to the robot namespace):
    lane_pose, intersection_navigation_pose (LanePose), wheels_cmd_executed (WheelsCmdStamped),
    stop_line_reading, obstacle_distance_reading (StopLineReading), fsm_node/mode (FSMState)
Publishes:
    car_cmd (Twist2DStamped)
"""
import math
from typing import Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from duckietown_msgs.msg import FSMState, LanePose, StopLineReading, Twist2DStamped, WheelsCmdStamped

from lane_control.controller import LaneController

# defaults: identical to dt-core (ROS 1) lane_controller_node/default.yaml
DEFAULT_PARAMETERS = {
    "v_bar": 0.19,
    "k_d_d": -2.0,
    "k_d_phi": -3.0,
    "k_I_d": -3.0,
    "theta_thres_max": 0.75,
    "theta_thres_min": -0.5,
    "d_thres": 0.2615,
    "d_offset": 0.0,
    "omega_ff": 0.0,
    "integral_bounds.d.top": 0.3,
    "integral_bounds.d.bot": -0.3,
    "integral_bounds.phi.top": 1.2,
    "integral_bounds.phi.bot": -1.2,
    "d_resolution": 0.011,
    "phi_resolution": 0.051,
    "stop_line_slowdown.start": 0.4,
    "stop_line_slowdown.end": 0.1,
    "verbose": 0,
}


class LaneControllerNode(Node):
    def __init__(self):
        super().__init__("lane_controller_node")

        # parameters (nested YAML mappings arrive as dotted names)
        for name, default in DEFAULT_PARAMETERS.items():
            self.declare_parameter(name, default)
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        self.controller = LaneController(self._build_parameters())

        # state
        self.fsm_state: Optional[str] = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.last_s: Optional[float] = None
        self.stop_line_distance: Optional[float] = None
        self.stop_line_detected: bool = False
        self.at_stop_line: bool = False
        self.obstacle_stop_line_distance: Optional[float] = None
        self.obstacle_stop_line_detected: bool = False
        self.at_obstacle_stop_line: bool = False
        self.current_pose_source: str = "lane_filter"

        # QoS: keep only the latest message (ROS 1 used queue_size=1), compatible with any publisher
        qos_latest = QoSProfile(depth=1)
        qos_latest.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_latest.history = HistoryPolicy.KEEP_LAST
        qos_cmd = QoSProfile(depth=1)
        qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        qos_cmd.history = HistoryPolicy.KEEP_LAST

        # publisher
        self.pub_car_cmd = self.create_publisher(Twist2DStamped, "car_cmd", qos_cmd)

        # subscribers
        self.create_subscription(LanePose, "lane_pose", lambda m: self.cb_all_poses(m, "lane_filter"), qos_latest)
        self.create_subscription(
            LanePose, "intersection_navigation_pose", lambda m: self.cb_all_poses(m, "intersection_navigation"), qos_latest
        )
        self.create_subscription(WheelsCmdStamped, "wheels_cmd_executed", self.cb_wheels_cmd_executed, qos_latest)
        self.create_subscription(StopLineReading, "stop_line_reading", self.cb_stop_line_reading, qos_latest)
        self.create_subscription(StopLineReading, "obstacle_distance_reading", self.cb_obstacle_stop_line_reading, qos_latest)
        self.create_subscription(FSMState, "fsm_node/mode", self.cb_mode, qos_latest)

        self.get_logger().info("Initialized! Parameters: %s" % self._build_parameters())

    # --- parameters -------------------------------------------------------

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _build_parameters(self) -> dict:
        return {
            "v_bar": float(self._p("v_bar")),
            "k_d_d": float(self._p("k_d_d")),
            "k_d_phi": float(self._p("k_d_phi")),
            "k_I_d": float(self._p("k_I_d")),
            "theta_thres_max": float(self._p("theta_thres_max")),
            "theta_thres_min": float(self._p("theta_thres_min")),
            "d_thres": float(self._p("d_thres")),
            "d_offset": float(self._p("d_offset")),
            "omega_ff": float(self._p("omega_ff")),
            "integral_bounds": {
                "d": {"top": float(self._p("integral_bounds.d.top")), "bot": float(self._p("integral_bounds.d.bot"))},
                "phi": {"top": float(self._p("integral_bounds.phi.top")), "bot": float(self._p("integral_bounds.phi.bot"))},
            },
            "d_resolution": float(self._p("d_resolution")),
            "phi_resolution": float(self._p("phi_resolution")),
            "stop_line_slowdown": {
                "start": float(self._p("stop_line_slowdown.start")),
                "end": float(self._p("stop_line_slowdown.end")),
            },
            "verbose": int(self._p("verbose")),
        }

    def _on_parameters_changed(self, _params) -> SetParametersResult:
        # the new values are applied after this callback returns; refresh the controller then
        self.create_timer(0.0, self._refresh_controller_parameters_once)
        return SetParametersResult(successful=True)

    def _refresh_controller_parameters_once(self) -> None:
        self.controller.update_parameters(self._build_parameters())
        self.get_logger().info("Parameters updated: %s" % self.controller.parameters)
        # one-shot timer: destroy the timer that called us
        for timer in list(self.timers):
            if timer.callback == self._refresh_controller_parameters_once:
                self.destroy_timer(timer)

    # --- subscribers --------------------------------------------------------

    def cb_obstacle_stop_line_reading(self, msg: StopLineReading) -> None:
        """Callback storing the current obstacle distance, if detected."""
        self.obstacle_stop_line_distance = float(np.sqrt(msg.stop_pose.x**2 + msg.stop_pose.y**2))
        self.obstacle_stop_line_detected = msg.stop_line_detected
        self.at_obstacle_stop_line = msg.at_stop_line
        if not self.obstacle_stop_line_detected:
            self.obstacle_stop_line_distance = None

    def cb_stop_line_reading(self, msg: StopLineReading) -> None:
        """Callback storing the distance to the next stop line, if detected."""
        self.stop_line_distance = -msg.stop_pose.x
        self.stop_line_detected = msg.stop_line_detected
        self.at_stop_line = msg.at_stop_line
        if not self.stop_line_detected:
            self.stop_line_distance = None

    def cb_mode(self, msg: FSMState) -> None:
        """Selects the pose source according to the FSM state."""
        self.fsm_state = msg.state
        if self.fsm_state == "INTERSECTION_CONTROL":
            self.current_pose_source = "intersection_navigation"
        else:
            self.current_pose_source = "lane_filter"
        if self.controller.parameters["verbose"] == 2:
            self.get_logger().info("Pose source: %s" % self.current_pose_source)

    def cb_all_poses(self, msg: LanePose, pose_source: str) -> None:
        """Computes a control command when the pose comes from the currently selected source."""
        if pose_source == self.current_pose_source:
            self.get_control_action(msg)

    def cb_wheels_cmd_executed(self, msg: WheelsCmdStamped) -> None:
        """Stores the last executed wheel command (used to pause integration while stopped)."""
        self.wheels_cmd_executed = msg

    # --- control ------------------------------------------------------------

    def get_control_action(self, pose_msg: LanePose) -> None:
        """Computes and publishes the control action for a lane pose (same logic as ROS 1)."""
        params = self.controller.parameters
        current_s = self.get_clock().now().nanoseconds * 1e-9
        dt = None if self.last_s is None else current_s - self.last_s

        if self.at_stop_line or self.at_obstacle_stop_line:
            v = 0.0
            omega = 0.0
        else:
            # errors
            d_err = pose_msg.d - params["d_offset"]
            phi_err = pose_msg.phi
            # cap the errors if they grow too large
            if abs(d_err) > params["d_thres"]:
                d_err = math.copysign(params["d_thres"], d_err)
            phi_err = min(max(phi_err, params["theta_thres_min"]), params["theta_thres_max"])

            wheels_cmd_exec = [self.wheels_cmd_executed.vel_left, self.wheels_cmd_executed.vel_right]
            if self.obstacle_stop_line_detected:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec, self.obstacle_stop_line_distance
                )
                # TODO: temporary fix (from ROS 1) for vehicle-detection latency: slow down near obstacles
                v = v * 0.25
                omega = omega * 0.25
            else:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec, self.stop_line_distance
                )
            # feed-forward action (i.e. during intersection navigation)
            omega += params["omega_ff"]

        cmd = Twist2DStamped()
        cmd.header = pose_msg.header
        cmd.v = float(v)
        cmd.omega = float(omega)
        self.pub_car_cmd.publish(cmd)
        self.last_s = current_s


def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
