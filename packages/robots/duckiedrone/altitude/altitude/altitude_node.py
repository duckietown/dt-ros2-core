"""ROS 2 altitude node for Duckiedrone.

Publishes the height of the drone above the floor [m] from the bottom
ToF, corrected for tilt with the IMU orientation. At startup, the mean
ToF reading while the drone rests on the floor is taken as the zero.

ROS parameters:
    calibration_seconds [s]: time on the floor before the zero is set.
    calibration_max_range [m]: readings above this restart calibration.
"""

import math
from collections.abc import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty

WARN_PERIOD = 5.0  # [s]


class AltitudeNode(Node):
    """Estimate the height above the floor from ToF range and IMU."""

    def __init__(self) -> None:
        """Initialize parameters, state, subscribers and publishers."""
        super().__init__("altitude_node")

        self._cal_seconds = self.declare_parameter(
            "calibration_seconds",
            3.0,
        ).value
        self._cal_max_range = self.declare_parameter(
            "calibration_max_range",
            0.10,
        ).value

        self._vertical_scale = 1.0
        self._offset: float | None = None  # [m], None until calibrated
        self._samples: list[float] = []
        self._cal_start: float | None = None

        self.create_subscription(Imu, "~/imu", self.imu_cb, 1)
        self.create_subscription(Range, "~/tof", self.tof_cb, 1)

        self._pub = self.create_publisher(Range, "~/altitude", 1)
        self._heartbeat = self.create_publisher(Empty, "~/heartbeat", 1)

    def imu_cb(self, msg: Imu) -> None:
        """Store the tilt factor cos(roll) * cos(pitch) from the IMU.

        It is R[2][2] of the quaternion's rotation matrix,
        1 - 2 * (qx^2 + qy^2): 1 when level, -1 upside down.
        """
        q = msg.orientation
        norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm2 == 0.0:
            self._vertical_scale = 1.0
            return

        self._vertical_scale = 1.0 - 2.0 * (q.x * q.x + q.y * q.y) / norm2

    def tof_cb(self, msg: Range) -> None:
        """Publish one height [m] and a heartbeat per valid ToF reading.

        height = range * tilt factor - zero, clamped at 0. The ToF
        header (stamp) is kept. Invalid readings publish nothing.
        """
        if not math.isfinite(msg.range) or msg.range > msg.max_range:
            self.get_logger().warn(
                "ToF sees no floor: no altitude published",
                throttle_duration_sec=WARN_PERIOD,
            )
            if self._offset is None:
                self._samples.clear()
                self._cal_start = None
            return

        if self._offset is None:
            self._calibrate(msg)
            return

        altitude = max((msg.range * self._vertical_scale) - self._offset, 0.0)

        altitude_msg = Range(
            header=msg.header,
            radiation_type=msg.radiation_type,
            field_of_view=msg.field_of_view,
            min_range=0.0,
            max_range=msg.max_range - self._offset,
            range=altitude,
        )
        self._pub.publish(altitude_msg)
        self._heartbeat.publish(Empty())

    def _calibrate(self, msg: Range) -> None:
        """Average floor readings over calibration_seconds as the zero.

        A reading above calibration_max_range restarts the count.
        """
        if msg.range > self._cal_max_range:
            self._samples.clear()
            self._cal_start = None
            self.get_logger().warn(
                "ToF above calibration_max_range: place the drone on the "
                "floor to calibrate",
                throttle_duration_sec=WARN_PERIOD,
            )
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._cal_start is None:
            self._cal_start = t
        self._samples.append(msg.range)

        if t - self._cal_start >= self._cal_seconds:
            self._offset = sum(self._samples) / len(self._samples)
            self.get_logger().info(
                f"Calibrated zero: {self._offset:.4f} m (mean of "
                f"{len(self._samples)} ToF readings)",
            )


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
