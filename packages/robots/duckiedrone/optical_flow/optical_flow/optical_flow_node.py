"""ROS 2 optical flow node for Duckiedrone."""

import numpy as np
import rclpy
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import Odometry
from opencv_apps.msg import FlowArrayStamped
from rclpy.node import Node
from sensor_msgs.msg import Range


class OpticalFlowNode(Node):
    """Convert motion vectors into optical-flow odometry."""

    def __init__(self) -> None:
        """Create publishers, subscriptions, and scaling state."""
        super().__init__("optical_flow_node")

        self.declare_parameter("process_frequency", 20)
        self.declare_parameter("base_homography_height", 0.063)
        self.declare_parameter("flow_scale", 0.90)
        self.declare_parameter("track_len", 10)
        self.declare_parameter("detect_interval", 5)
        self.declare_parameter("img_scale", 1.0)

        self._range = 1.0
        base_height = float(self.get_parameter("base_homography_height").value)
        self._scale = self._range / base_height

        self.pub_odometry = self.create_publisher(
            Odometry,
            "~/visual_odometry",
            1,
        )
        self.pub_seg_list = self.create_publisher(
            SegmentList,
            "~/lineseglist_out",
            1,
        )
        self.pub_debug_raw_odometry = self.create_publisher(
            Odometry,
            "~/debug/raw_odometry",
            1,
        )

        self.create_subscription(
            FlowArrayStamped,
            "~/motion_vectors",
            self.cb_motion_vectors,
            1,
        )
        self.create_subscription(Range, "~/range", self.cb_new_range, 1)
        self.create_subscription(
            SegmentList,
            "~/projected_motion_vectors",
            self.cb_projected_motion_vectors,
            1,
        )

    def cb_motion_vectors(self, msg: FlowArrayStamped) -> None:
        """Repackage motion vectors for ground projection."""
        segment_list = SegmentList()
        segment_list.header = msg.header

        if msg.flow is None:
            self.get_logger().warning("No motion vectors received.")
            return

        segment_list.segments = []
        for flow_vector in msg.flow:
            start_point = PointMsg(
                x=flow_vector.point.x,
                y=flow_vector.point.y,
            )
            end_point = PointMsg(
                x=flow_vector.point.x + flow_vector.velocity.x,
                y=flow_vector.point.y + flow_vector.velocity.y,
            )
            segment = Segment(
                points=[start_point, end_point],
            )
            segment_list.segments.append(segment)

        if (
            self.pub_debug_raw_odometry.get_subscription_count() > 0
            and msg.flow
        ):
            flow_scale = float(self.get_parameter("flow_scale").value)
            mean_velocity_x = np.mean(
                [flow_vector.velocity.x for flow_vector in msg.flow],
            )
            mean_velocity_y = np.mean(
                [flow_vector.velocity.y for flow_vector in msg.flow],
            )
            odometry_msg = Odometry()
            odometry_msg.header = msg.header
            odometry_msg.child_frame_id = "camera"
            odometry_msg.twist.twist.linear.x = float(
                mean_velocity_x * flow_scale * self._range,
            )
            odometry_msg.twist.twist.linear.y = float(
                mean_velocity_y * flow_scale * self._range,
            )
            self.pub_debug_raw_odometry.publish(odometry_msg)

        self.pub_seg_list.publish(segment_list)

    def cb_projected_motion_vectors(self, msg: SegmentList) -> None:
        """Compute velocity from projected motion vectors."""
        if msg.segments is None:
            self.get_logger().warning("Empty motion vectors array received.")
            return

        num_motion_vectors = len(msg.segments)
        if num_motion_vectors == 0:
            return

        motion_vectors = np.zeros((2, num_motion_vectors))
        for index, flow in enumerate(msg.segments):
            motion_vectors[:, index] = np.array(
                [
                    flow.points[1].x - flow.points[0].x,
                    flow.points[1].y - flow.points[0].y,
                ],
            )

        velocity = np.mean(motion_vectors, axis=1)
        velocity = np.array([velocity[1], velocity[0]])

        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = msg.header.frame_id
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.twist.twist.linear.x = float(velocity[0])
        odometry_msg.twist.twist.linear.y = float(velocity[1])
        self.pub_odometry.publish(odometry_msg)

    def cb_new_range(self, msg: Range) -> None:
        """Update the current altitude scaling factor."""
        self._range = msg.range
        base_height = float(self.get_parameter("base_homography_height").value)
        self._scale = self._range / base_height


def main(args: list[str] | None = None) -> None:
    """Run the optical flow node."""
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
