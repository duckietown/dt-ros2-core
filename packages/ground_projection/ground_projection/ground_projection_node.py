#!/usr/bin/env python3

import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from dt_computer_vision.camera import CameraModel
from dt_computer_vision.camera.homography import Homography, HomographyToolkit
from dt_computer_vision.camera.types import (
    NormalizedImagePoint,
    Pixel,
    ResolutionIndependentImagePoint,
)
from dt_computer_vision.ground_projection import GroundPoint, GroundProjector
from dt_computer_vision.ground_projection.rendering import (
    debug_image,
)
from duckietown_msgs.msg import Segment as SegmentMsg
from duckietown_msgs.msg import SegmentList
from geometry_msgs.msg import Point as PointMsg
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class GroundProjectionNode(Node):
    """Node that projects image line segments onto the ground plane in the robot's
    reference frame using the camera extrinsics (homography). This enables 2D ground-plane lane
    localization.

    Args:
        node_name (str): Unique name for this ROS 2 node.

    Subscriptions (relative to node namespace):
        camera_info (sensor_msgs.msg.CameraInfo): Camera intrinsics used for rectification.
        lineseglist_in (duckietown_msgs.msg.SegmentList): Line segments in pixel space (unrectified input).

    Publications (relative to node namespace):
        lineseglist_out (duckietown_msgs.msg.SegmentList): Segments projected on the ground plane
            relative to the robot.
        debug/ground_projection_image/compressed (sensor_msgs.msg.CompressedImage): Debug image showing
            the robot relative to projected segments (checks extrinsic accuracy).
        debug/projected_image/rectified/compressed (sensor_msgs.msg.CompressedImage): Rectified image
            (checks rectification accuracy).
        debug/projected_image/compressed (sensor_msgs.msg.CompressedImage): Image after homography
            (checks homography accuracy).

    """

    bridge: CvBridge
    projector: GroundProjector | None = None

    def __init__(self, node_name: str):
        # Initialize ROS 2 Node
        super().__init__(node_name)

        self.bridge = CvBridge()
        self.projector: GroundProjector | None = None
        self.camera: CameraModel | None = None
        self.homography: Homography | None = None
        self._first_processing_done = False
        self.camera_info_received = False

        # subscribers (relative names; resolve under node namespace)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, "camera_info", self.cb_camera_info, 10,
        )
        self.sub_lineseglist_ = self.create_subscription(
            SegmentList, "lineseglist_in", self.lineseglist_cb, 10,
        )

        # publishers
        self.pub_lineseglist = self.create_publisher(
            SegmentList, "lineseglist_out", 10,
        )
        self.pub_debug_road_view_img = self.create_publisher(
            CompressedImage, "debug/ground_projection_image/compressed", 10,
        )

        self.pub_debug_rectified_img = self.create_publisher(
            CompressedImage, "debug/projected_image/rectified/compressed", 10,
        )

        self.pub_debug_projected_img = self.create_publisher(
            CompressedImage, "debug/projected_image/compressed", 10,
        )

        self.bridge = CvBridge()

        self.debug_img_bg = None


    def cb_camera_info(self, msg: CameraInfo):
        """Initializes a :py:class:`image_processing.GroundProjectionGeometry` object and a
        :py:class:`image_processing.Rectify` object for image rectification

        Args:
            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.

        """
        if not self.camera_info_received:
            self.get_logger().info("Received camera info message")
            # create camera object
            self.camera = CameraModel(
                width=msg.width,
                height=msg.height,
                K=np.reshape(msg.k, (3, 3)),
                D=np.reshape(msg.d, (5,)),
                P=np.reshape(msg.p, (3, 4)),
            )

            self.homography = self.load_extrinsics()
            self.get_logger().info(f"Got homography {self.homography}")
            self.camera.H = self.homography
            self.projector = GroundProjector(self.camera)

            self.get_logger().info("Camera model initialized")

        self.camera_info_received = True

    def _pixel_to_ground(self, p: ResolutionIndependentImagePoint) -> GroundPoint:
        """Converts a pixel coordinate to a ground point.

        Args:
            p (:obj:`dt_computer_vision.camera.Pixel`): Pixel coordinate

        Returns:
            :obj:`dt_computer_vision.ground_projection`: Ground point

        """
        if self.camera is None:
            raise ValueError("Camera model not initialized")

        pixel: Pixel = self.camera.independent2pixel(p)
        rect: Pixel = self.camera.rectifier.rectify_pixel(pixel)
        vector: NormalizedImagePoint = self.camera.pixel2vector(rect)
        p_ground: GroundPoint = self.projector.vector2ground(vector)

        return p_ground

    def pixel_msg_to_ground_msg(self, pixel_msg: PointMsg) -> PointMsg:
        """Converts a pixel message to a ground message.

        Args:
            pixel_msg (:obj:`geometry_msgs.msg.Point`): Pixel message

        Returns:
            :obj:`geometry_msgs.msg.Point`: Ground message

        """
        p = ResolutionIndependentImagePoint(x=pixel_msg.x, y=pixel_msg.y)
        p_ground = self._pixel_to_ground(p)
        return PointMsg(x=p_ground.x, y=p_ground.y)

    def lineseglist_cb(self, seglist_msg: SegmentList):
        """Projects a list of line segments on the ground reference frame point by point by
        calling :py:meth:`pixel_msg_to_ground_msg`. Then publishes the projected list of segments.

        Args:
            seglist_msg (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in pixel space from
            unrectified images

        """
        if self.camera_info_received:
            # the list of segments on the ground that we will publish
            seglist_out = SegmentList()
            seglist_out.header = seglist_msg.header
            colored_segments = {(255, 255, 255): [], (0,255,255): [], (255,0,0):[]}

            for received_segment in seglist_msg.segments:
                received_segment: SegmentMsg
                projected_segment = SegmentMsg()

                projected_segment.points[0] = self.pixel_msg_to_ground_msg(
                    received_segment.pixels_normalized[0],
                )
                projected_segment.points[1] = self.pixel_msg_to_ground_msg(
                    received_segment.pixels_normalized[1],
                )
                projected_segment.color = received_segment.color
                seglist_out.segments.append(projected_segment)

                if projected_segment.color == 0:
                    color_vect = (255,255,255)
                elif projected_segment.color == 1:
                    color_vect = (0, 255, 255)
                else:
                    color_vect = (255, 0, 0)
                colored_segments[color_vect].append((projected_segment.points[0], projected_segment.points[1]))
            self.pub_lineseglist.publish(seglist_out)

            if not self._first_processing_done:
                self.get_logger().info("First projected segments published.")
                self._first_processing_done = True

            if self.pub_debug_road_view_img.get_subscription_count() > 0:
                debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(
                    debug_image(colored_segments,(300, 300), grid_size=6, s_segment_thickness=5),
                )
                debug_image_msg.header = seglist_out.header
                self.pub_debug_road_view_img.publish(debug_image_msg)
        else:
            self.get_logger().warning("Waiting for a CameraInfo message")

    def load_extrinsics(self) -> Homography | None:
        """Loads the homography matrix from the extrinsic calibration file.

        Returns:
            :obj:`Homography`: the loaded homography matrix

        """
        # load extrinsic calibration
        cali_file_folder = "/data/config/calibrations/camera_extrinsic/"
        cali_file = cali_file_folder + self.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.get_logger().warning(
                f"Can't find calibration file: {cali_file}\n Using default calibration instead.",
            )
            cali_file = os.path.join(cali_file_folder, "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = "Found no calibration file ... aborting"
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        try:
            H: Homography = HomographyToolkit.load_from_disk(
                cali_file, return_date=False,
            )  # type: ignore
            return H.reshape((3, 3))
        except Exception as e:
            msg = f"Error in parsing calibration file {cali_file}:\n{e}"
            self.get_logger().error(msg)
            raise


def main():
    rclpy.init()
    node = GroundProjectionNode(node_name="ground_projection_node")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
