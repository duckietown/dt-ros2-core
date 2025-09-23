#!/usr/bin/env python3
"""ROS 2 lane filter node."""
from __future__ import annotations

from typing import List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from cv_bridge import CvBridge

from dt_state_estimation.lane_filter import LaneFilterHistogram
from dt_state_estimation.lane_filter.types import Segment, SegmentColor, SegmentPoint
from dt_state_estimation.lane_filter.rendering import plot_d_phi

from duckietown_msgs.msg import EpisodeStart, LanePose, SegmentList, WheelEncoderStamped
from duckietown_msgs.msg import Segment as SegmentMsg
from sensor_msgs.msg import CompressedImage


class LaneFilterNode(Node):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the
    center of the lane. It receives the segments extracted by the line detector as input and publishes the
    resulting lane pose estimate.

    Configuration (ROS 2 parameters):
        debug (:obj:`bool`): Enable/disable publishing of debug topics and images.
        predict_frequency (:obj:`float`): Frequency for running the prediction step.
        lane_filter_histogram_configuration (:obj:`dict`): Parameters for the lane pose estimation filter.

    Subscribers:
        segment_list (:obj:`SegmentList`): Detected line segments from the line detector.
        left_wheel_encoder_driver_node/tick (:obj:`WheelEncoderStamped`): Left wheel encoder ticks.
        right_wheel_encoder_driver_node/tick (:obj:`WheelEncoderStamped`): Right wheel encoder ticks.
        episode_start (:obj:`EpisodeStart`): Signal for starting a new episode (resets the filter).

    Publishers:
        lane_pose (:obj:`LanePose`): The computed lane pose estimate.
        debug/belief_img/compressed (:obj:`CompressedImage`): Debug image showing the filter's internal state.
        debug/plot_d_phi/compressed (:obj:`CompressedImage`): Plot of the belief over d/phi.

    """

    def __init__(self) -> None:
        super().__init__('lane_filter_node')

        self._debug = self.declare_parameter('debug', True).value
        self._predict_freq = float(self.declare_parameter('predict_frequency', 30.0).value)
        filter_config = self.declare_parameter('lane_filter_histogram_configuration', None).value or {}
        encoder_resolution = int(self.declare_parameter('encoder_resolution', 135).value)
        wheel_baseline = float(self.declare_parameter('wheel_baseline', 0.0).value)
        wheel_radius = float(self.declare_parameter('wheel_radius', 0.0).value)

        if not isinstance(filter_config, dict):
            self.get_logger().warn(
                'Parameter lane_filter_histogram_configuration is not a dictionary; using empty defaults.'
            ) # FIXME: this does not seem to use the parameters specified in the config file
            filter_config = {}

        filter_config.setdefault('encoder_resolution', encoder_resolution)
        if wheel_baseline:
            filter_config.setdefault('wheel_baseline', wheel_baseline)
        if wheel_radius:
            filter_config.setdefault('wheel_radius', wheel_radius)

        try:
            self.filter = LaneFilterHistogram(**filter_config)
        except TypeError as error:
            self.get_logger().error(f'Failed to construct LaneFilterHistogram: {error}')
            raise

        self.bridge = CvBridge()

        qos = QoSProfile(depth=1)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.left_encoder_ticks = 0
        self.left_encoder_initialized = False
        self.left_encoder_ticks_delta = 0

        self.right_encoder_ticks = 0
        self.right_encoder_initialized = False
        self.right_encoder_ticks_delta = 0

        self.last_segment_header: Optional['std_msgs.msg.Header'] = None

        self.create_subscription(SegmentList, 'segment_list', self.cb_process_segments, qos)
        self.create_subscription(
            WheelEncoderStamped,
            'left_wheel_encoder_driver_node/tick',
            self.cb_process_left_encoder,
            qos,
        )
        self.create_subscription(
            WheelEncoderStamped,
            'right_wheel_encoder_driver_node/tick',
            self.cb_process_right_encoder,
            qos,
        )
        self.create_subscription(EpisodeStart, 'episode_start', self.cb_episode_start, qos)

        self.pub_lane_pose = self.create_publisher(LanePose, 'lane_pose', qos)
        self.pub_belief_img = self.create_publisher(CompressedImage, 'debug/belief_img/compressed', qos)
        self.pub_plot_d_phi = self.create_publisher(CompressedImage, 'debug/plot_d_phi/compressed', qos)

        if self._predict_freq > 0.0:
            self.create_timer(1.0 / self._predict_freq, self.cb_predict)
        else:
            self.get_logger().warn('predict_frequency <= 0.0, encoder prediction disabled.')

        self.publish_estimate(self.last_segment_header)

    def cb_episode_start(self, _: EpisodeStart) -> None:
        self.get_logger().info('Episode start received, resetting filter state.')
        self.filter.initialize()

    def cb_process_left_encoder(self, msg: WheelEncoderStamped) -> None:
        if not self.left_encoder_initialized:
            self.left_encoder_ticks = msg.data
            self.left_encoder_initialized = True
        self.left_encoder_ticks_delta = msg.data - self.left_encoder_ticks

    def cb_process_right_encoder(self, msg: WheelEncoderStamped) -> None:
        if not self.right_encoder_initialized:
            self.right_encoder_ticks = msg.data
            self.right_encoder_initialized = True
        self.right_encoder_ticks_delta = msg.data - self.right_encoder_ticks

    def cb_predict(self) -> None:
        if self.left_encoder_ticks_delta == 0 and self.right_encoder_ticks_delta == 0:
            return

        self.filter.predict(self.left_encoder_ticks_delta, self.right_encoder_ticks_delta)
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

        self.publish_estimate(self.last_segment_header)

    def cb_process_segments(self, msg: SegmentList) -> None:
        self.cb_predict()
        self.last_segment_header = msg.header

        dt_segment_list: List[Segment] = []
        for segment in msg.segments:
            dt_segment_list.append(self._convert_segment(segment))

        self.filter.update(dt_segment_list)
        self.publish_estimate(msg.header)

    def _convert_segment(self, segment: SegmentMsg) -> Segment:
        color = SegmentColor.WHITE
        if segment.color == SegmentMsg.YELLOW:
            color = SegmentColor.YELLOW
        elif segment.color == SegmentMsg.RED:
            color = SegmentColor.RED

        points = [
            SegmentPoint(x=segment.points[0].x, y=segment.points[0].y),
            SegmentPoint(x=segment.points[1].x, y=segment.points[1].y),
        ]
        return Segment(color=color, points=points)

    def publish_estimate(self, header) -> None:
        d_max, phi_max = self.filter.get_estimate()
        in_lane = self.filter.get_max() > self.filter.min_max

        lane_pose = LanePose()
        if header is not None:
            lane_pose.header = header
        else:
            lane_pose.header.stamp = self.get_clock().now().to_msg()
        lane_pose.d = float(d_max)
        lane_pose.phi = float(phi_max)
        lane_pose.in_lane = bool(in_lane)
        lane_pose.status = LanePose.NORMAL

        self.pub_lane_pose.publish(lane_pose)

        if self._debug:
            self.publish_debug_outputs(header)

    def publish_debug_outputs(self, header) -> None:
        if self.pub_plot_d_phi.get_subscription_count() == 0:
            return

        d_max, phi_max = self.filter.get_estimate()
        img = plot_d_phi(d=d_max, phi=phi_max)
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(img)
        except Exception as error:
            self.get_logger().warn(f'Failed to convert debug image: {error}')
            return

        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_plot_d_phi.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
