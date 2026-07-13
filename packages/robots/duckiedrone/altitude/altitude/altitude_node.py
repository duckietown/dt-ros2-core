"""ROS 2 altitude node for Duckiedrone.

Computes ground altitude from a downward-facing ToF sensor,
corrected by the vehicle tilt inferred from the IMU orientation.
"""

import math
from collections.abc import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty


class AltitudeNode(Node):
    """Estimate altitude from ToF range and IMU orientation."""

    def __init__(self) -> None:
        """Initialize subscriptions, publishers, and parameters."""
        super().__init__("altitude_node")

        self.declare_parameter("h_offset", 0.02)

        self._vertical_scale = 1.0
        self._last_range = 0.0

        self.create_subscription(Imu, "~/imu", self.imu_cb, 1)
        self.create_subscription(Range, "~/tof", self.tof_cb, 1)

        self._pub = self.create_publisher(Range, "~/altitude", 1)
        self._heartbeat = self.create_publisher(Empty, "~/heartbeat", 1)

    def imu_cb(self, msg: Imu) -> None:
        """Update the tilt correction factor from the IMU quaternion."""
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            self._vertical_scale = 1.0
            return

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        vertical_scale = 1.0 - 2.0 * (qx * qx + qy * qy)
        self._vertical_scale = max(-1.0, min(1.0, vertical_scale))

    def tof_cb(self, msg: Range) -> None:
        """Publish tilt-corrected altitude and refresh the heartbeat."""
        range_value = msg.range
        if range_value > msg.max_range:
            if self._last_range == 0.0:
                return
            range_value = self._last_range

        self._last_range = range_value

        h_offset = float(self.get_parameter("h_offset").value)
        range_value = max(range_value - h_offset, 0.0)
        altitude = range_value * self._vertical_scale

        altitude_msg = Range(
            header=msg.header,
            radiation_type=msg.radiation_type,
            field_of_view=msg.field_of_view,
            min_range=msg.min_range,
            max_range=msg.max_range,
            range=altitude,
        )
        self._pub.publish(altitude_msg)
        self._heartbeat.publish(Empty())


def main(args: Sequence[str] | None = None) -> None:
    """Run the altitude node until shutdown."""
    rclpy.init(args=args)
    node = AltitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
