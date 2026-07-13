"""ROS 2 simple state estimator for Duckiedrone."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty


class SimpleStateEstimatorNode(Node):
    """Estimate altitude-only state from the altitude node output."""

    def __init__(self) -> None:
        """Initialize publishers and subscriptions for simple altitude state."""
        super().__init__("state_estimator_node")

        self._heartbeat_pub = self.create_publisher(Empty, "~/heartbeat", 1)
        self._state_pub = self.create_publisher(Odometry, "~/state", 1)
        self.create_subscription(Range, "~/altitude", self._altitude_cb, 1)

    def _altitude_cb(self, msg: Range) -> None:
        """Publish a simple odometry message containing only altitude.

        The heartbeat is emitted alongside the state update.
        """
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = "map"
        odometry.pose.pose.position.z = msg.range
        self._state_pub.publish(odometry)
        self._heartbeat_pub.publish(Empty())


def main(args: list[str] | None = None) -> None:
    """Run the simple state estimator node."""
    rclpy.init(args=args)
    node = SimpleStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()