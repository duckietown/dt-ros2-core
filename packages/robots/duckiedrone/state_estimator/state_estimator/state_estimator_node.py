"""ROS 2 EMA-based state estimator for Duckiedrone."""

from math import asin, atan2, cos

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty


class StateEstimatorNode(Node):
    """Estimate drone state from IMU, altitude, pose, and flow data."""

    def __init__(self) -> None:
        """Initialize EMA state estimation and ROS interfaces."""
        super().__init__("state_estimator_node")

        self.declare_parameter("primary", "ema")
        self.declare_parameter("others", ["ema", "mocap"])
        self.declare_parameter("ir_throttled", value=False)
        self.declare_parameter("imu_throttled", value=False)
        self.declare_parameter("optical_flow_throttled", value=False)
        self.declare_parameter("camera_pose_throttled", value=False)
        self.declare_parameter("sdim", 1)
        self.declare_parameter("student_ukf", value=False)
        self.declare_parameter("ir_var", 0.0)
        self.declare_parameter("state_rate", 30.0)
        self.declare_parameter("ema_alpha_pose", 0.2)
        self.declare_parameter("ema_alpha_twist", 0.1)
        self.declare_parameter("ema_alpha_range", 0.1)

        primary = str(self.get_parameter("primary").value)
        if primary != "ema":
            message = (
                f"state_estimator primary '{primary}' is not ported yet; "
                "use 'ema'."
            )
            raise NotImplementedError(
                message,
            )

        self.state_msg = Odometry()
        self.state_msg.header.frame_id = "Body"
        self._received_imu = False
        self._received_range = False

        self.heartbeat_pub = self.create_publisher(Empty, "~/heartbeat", 1)
        self.state_pub = self.create_publisher(Odometry, "~/state", 1)

        self.create_subscription(Imu, "~/imu", self.imu_cb, 1)
        self.create_subscription(Range, "~/range", self.range_cb, 1)
        self.create_subscription(Odometry, "~/twist", self.twist_cb, 1)
        self.create_subscription(PoseStamped, "pose_topic", self.pose_cb, 1)

        state_rate = float(self.get_parameter("state_rate").value)
        state_rate = max(state_rate, 1e-3)
        self._timer = self.create_timer(1.0 / state_rate, self.state_callback)

    def pose_cb(self, msg: PoseStamped) -> None:
        """Apply EMA smoothing to x/y pose updates."""
        alpha = float(self.get_parameter("ema_alpha_pose").value)
        position = self.state_msg.pose.pose.position
        position.x = (1.0 - alpha) * position.x + alpha * msg.pose.position.x
        position.y = (1.0 - alpha) * position.y + alpha * msg.pose.position.y

    def range_cb(self, msg: Range) -> None:
        """Apply EMA smoothing to the corrected altitude estimate."""
        alpha = float(self.get_parameter("ema_alpha_range").value)
        roll, pitch = self._roll_pitch_from_orientation()
        current_altitude = msg.range * cos(roll) * cos(pitch)
        previous_altitude = self.state_msg.pose.pose.position.z
        smoothed_altitude = (
            (1.0 - alpha) * current_altitude + alpha * previous_altitude
        )
        self.state_msg.pose.pose.position.z = max(0.0, smoothed_altitude)
        self._received_range = True

    def imu_cb(self, msg: Imu) -> None:
        """Store the latest IMU orientation in the published state."""
        self.state_msg.pose.pose.orientation = msg.orientation
        self._received_imu = True

    def twist_cb(self, msg: Odometry) -> None:
        """Apply EMA smoothing to flow-based linear velocity."""
        alpha = float(self.get_parameter("ema_alpha_twist").value)
        velocity = self.state_msg.twist.twist.linear
        new_velocity = msg.twist.twist.linear
        velocity.x = self._near_zero(
            (1.0 - alpha) * velocity.x + alpha * new_velocity.x,
        )
        velocity.y = self._near_zero(
            (1.0 - alpha) * velocity.y + alpha * new_velocity.y,
        )
        velocity.z = self._near_zero(
            (1.0 - alpha) * velocity.z + alpha * new_velocity.z,
        )
        self.state_msg.twist.twist.angular = msg.twist.twist.angular

    def state_callback(self) -> None:
        """Republish the current EMA state after core inputs arrive."""
        self.heartbeat_pub.publish(Empty())

        if not (self._received_imu and self._received_range):
            return

        self.state_msg.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.state_msg)

    def _roll_pitch_from_orientation(self) -> tuple[float, float]:
        """Convert the current quaternion to roll and pitch."""
        orientation = self.state_msg.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = asin(sinp)
        return roll, pitch

    @staticmethod
    def _near_zero(value: float, epsilon: float = 1e-6) -> float:
        """Zero out tiny floating-point values for readability."""
        return value if abs(value) > epsilon else 0.0


def main(args: list[str] | None = None) -> None:
    """Run the EMA state estimator node."""
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
