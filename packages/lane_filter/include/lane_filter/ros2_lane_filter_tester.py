#!/usr/bin/env python3
"""ROS 2 tester node for the lane filter."""
from __future__ import annotations

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from duckietown_msgs.msg import Segment, SegmentList


class LaneFilterTesterNode(Node):
    """Publishes a synthetic segment list for testing the filter."""

    def __init__(self) -> None:
        super().__init__('lane_filter_tester_node')

        self.declare_parameter('x1', 0.0)
        self.declare_parameter('y1', 0.0)
        self.declare_parameter('x2', 0.0)
        self.declare_parameter('y2', 0.0)
        self.declare_parameter('color', 'white')

        qos = QoSProfile(depth=1)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(SegmentList, 'segment_list', qos)
        self._published = False
        self.create_timer(0.1, self._publish_once)

    def _publish_once(self) -> None:
        if self._published:
            return

        seg = Segment()
        seg.points[0].x = float(self.get_parameter('x1').value)
        seg.points[0].y = float(self.get_parameter('y1').value)
        seg.points[1].x = float(self.get_parameter('x2').value)
        seg.points[1].y = float(self.get_parameter('y2').value)

        color = str(self.get_parameter('color').value).lower()
        if color == 'yellow':
            seg.color = Segment.YELLOW
        elif color == 'red':
            seg.color = Segment.RED
        else:
            seg.color = Segment.WHITE

        seg_list = SegmentList()
        seg_list.segments.append(seg)
        self.publisher.publish(seg_list)
        self._published = True
        self.get_logger().info('Published test segment list.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneFilterTesterNode()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not node._published:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
