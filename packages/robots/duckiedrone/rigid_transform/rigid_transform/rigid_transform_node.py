"""ROS 2 rigid-transform visual odometry node for Duckiedrone."""

from math import atan2, cos, sin, sqrt
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class RigidTransformNode(Node):
    """Estimate planar motion using frame-to-frame affine transforms."""

    def __init__(self) -> None:
        super().__init__("rigid_transform_node")
        self.bridge = CvBridge()
        self.pose_msg = PoseStamped()
        self.altitude = 0.03
        self.altitude_ts = time.monotonic()
        self.x_position_from_state = 0.0
        self.y_position_from_state = 0.0
        self.position_control = True
        self.previous_image = None
        self.consecutive_lost_counter = 0
        self.lost = False
        self.previous_points = None

        self._posepub = self.create_publisher(PoseStamped, "~/pose", 1)
        self._lostpub = self.create_publisher(Bool, "~/lost", 1)
        self._debug_img_pub = self.create_publisher(CompressedImage, "~/debug_image", 10)

        self.create_subscription(CompressedImage, "~/image/compressed", self.image_callback, 1)
        self.create_subscription(Range, "~/range", self.altitude_cb, 1)
        self.create_subscription(Odometry, "~/state", self.state_callback, 1)

        self.create_service(Trigger, "~/reset_transform", self.reset_callback)
        self.create_service(SetBool, "~/position_control", self.position_control_callback)

    def altitude_cb(self, msg: Range) -> None:
        self.altitude = msg.range
        self.altitude_ts = time.monotonic()

    def image_callback(self, msg: CompressedImage) -> None:
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.check_altitude_timeout()

        if self.position_control:
            if self.previous_image is None:
                self.previous_image = image
                self.previous_points = self.detect_features(image)
            else:
                self.process_image(image)

        self.publish_lost_status()

    def check_altitude_timeout(self) -> None:
        duration = time.monotonic() - self.altitude_ts
        if duration > 10.0:
            self.get_logger().warning(f"No altitude received for {duration:10.4f} seconds.")

    def process_image(self, image) -> None:
        current_points = self.detect_features(image)
        if self.previous_points is None or current_points is None:
            self.handle_lost_image()
            self.previous_image = image
            self.previous_points = current_points
            return

        transform, _ = cv2.estimateAffinePartial2D(self.previous_points, current_points)
        if transform is not None:
            self.lost = False
            self.update_pose_with_transform(transform)
        else:
            self.handle_lost_image()

        self.previous_image = image
        self.previous_points = current_points

        if self._debug_img_pub.get_subscription_count() > 0 and current_points is not None:
            self.publish_debug_image(image.copy(), current_points)

    def publish_debug_image(self, image, points) -> None:
        for point in points:
            x, y = point.ravel()
            cv2.circle(image, (int(x), int(y)), 3, (0, 255, 0), -1)
        msg = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")
        self._debug_img_pub.publish(msg)

    def detect_features(self, image):
        if self.previous_points is None:
            return cv2.goodFeaturesToTrack(image, maxCorners=10, qualityLevel=0.01, minDistance=8)
        next_points, _status, _err = cv2.calcOpticalFlowPyrLK(self.previous_image, image, self.previous_points, None)
        return next_points

    def update_pose_with_transform(self, transform: np.ndarray) -> None:
        translation, yaw = self.translation_and_yaw(transform)
        self.pose_msg.pose.position.x += translation[0] * self.altitude
        self.pose_msg.pose.position.y += translation[1] * self.altitude
        self.pose_msg.pose.orientation.z = sin(yaw / 2.0)
        self.pose_msg.pose.orientation.w = cos(yaw / 2.0)

    def handle_lost_image(self) -> None:
        self.get_logger().warning("Lost image!")
        if self.lost:
            self.consecutive_lost_counter += 1
        else:
            self.lost = True

    def publish_lost_status(self) -> None:
        if self.lost and self.consecutive_lost_counter >= 10:
            self._lostpub.publish(Bool(data=True))
            self.consecutive_lost_counter = 0
        else:
            self.consecutive_lost_counter = 0
            self._lostpub.publish(Bool(data=False))

        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self._posepub.publish(self.pose_msg)

    def translation_and_yaw(self, transform: np.ndarray):
        tx = float(transform[0, 2])
        ty = float(transform[1, 2])
        translation_x_y = [ty / 640.0, tx / 480.0]
        yaw_scale = sqrt(transform[0, 0] ** 2 + transform[1, 0] ** 2)
        yaw = atan2(float(transform[1, 0]) / yaw_scale, float(transform[0, 0]) / yaw_scale)
        return translation_x_y, yaw

    def reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.previous_image = None
        self.previous_points = None
        self.pose_msg = PoseStamped()
        self._lostpub.publish(Bool(data=False))
        response.success = True
        response.message = "Pose reset complete"
        return response

    def position_control_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.position_control = request.data
        response.success = True
        response.message = "Position control enabled" if request.data else "Position control disabled"
        return response

    def state_callback(self, msg: Odometry) -> None:
        self.altitude = msg.pose.pose.position.z
        self.x_position_from_state = msg.pose.pose.position.x
        self.y_position_from_state = msg.pose.pose.position.y


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RigidTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()