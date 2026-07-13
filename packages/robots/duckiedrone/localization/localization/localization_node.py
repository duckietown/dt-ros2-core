"""ROS 2 Monte-Carlo localization node for Duckiedrone."""

from math import cos, sin
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .localization_helper.helper import PROB_THRESHOLD, LocalizationParticleFilter

MAP_PIXEL_WIDTH = 3227
MAP_PIXEL_HEIGHT = 2447
MAP_REAL_WIDTH = 1.4
MAP_REAL_HEIGHT = 1.07
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 640
METER_TO_PIXEL = ((float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH) + (float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT)) / 2.0
CAMERA_CENTER = np.array([(CAMERA_WIDTH - 1) / 2.0, (CAMERA_HEIGHT - 1) / 2.0], dtype=np.float32).reshape(-1, 1, 2)
MAX_BAD_COUNT = -10
NUM_PARTICLE = 30
NUM_FEATURES = 200


class LocalizationNode(Node):
    """Monte-Carlo localization using downward-facing camera features."""

    def __init__(self) -> None:
        super().__init__("localization_node")
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.detector = cv2.ORB.create(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)

        map_image_path = Path(__file__).resolve().parents[5] / "assets" / "localization" / "map.jpg"
        self.estimator = LocalizationParticleFilter(
            camera_width=CAMERA_WIDTH,
            camera_height=CAMERA_HEIGHT,
            camera_center=CAMERA_CENTER,
            map_image_path=str(map_image_path),
        )

        self.pos = [0.0, 0.0, 0.0, 0.0]
        self.posemsg = PoseStamped()
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.z = 0.0
        self.first_locate = True
        self.locate_position = False
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None
        self.map_counter = 0
        self.max_map_counter = 0
        self.alpha_yaw = 0.1
        self.hybrid_alpha = 0.3

        self.create_service(Trigger, "~/reset_transform", self.reset_callback)
        self.create_subscription(Odometry, "~/state", self.state_callback, 1)
        self.create_subscription(CompressedImage, "~/image", self.image_callback, 1)
        self.posepub = self.create_publisher(PoseStamped, "~/pose", 1)

    def image_callback(self, msg: CompressedImage) -> None:
        try:
            curr_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as error:
            self.get_logger().error(f"Failed to convert image: {error}")
            return

        now = self.get_clock().now().to_msg()
        self.posemsg.header.stamp = now

        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)
            if curr_kp is not None and curr_des is not None and len(curr_kp) > 0:
                if self.first_locate:
                    particle = self.estimator.initialize_particles(NUM_PARTICLE, curr_kp, curr_des)
                    self.first_locate = False
                    self.pos = [particle.x(), particle.y(), particle.z(), particle.yaw()]
                    self._update_pose_msg(*self.pos)
                else:
                    particle = self.estimator.update(
                        self.z,
                        self.angle_x,
                        self.angle_y,
                        self.prev_kp,
                        self.prev_des,
                        curr_kp,
                        curr_des,
                    )
                    self.pos = [
                        self.hybrid_alpha * particle.x() + (1.0 - self.hybrid_alpha) * self.pos[0],
                        self.hybrid_alpha * particle.y() + (1.0 - self.hybrid_alpha) * self.pos[1],
                        self.z,
                        self.alpha_yaw * particle.yaw() + (1.0 - self.alpha_yaw) * self.pos[3],
                    ]
                    self._update_pose_msg(*self.pos)
                    self.posepub.publish(self.posemsg)
                    if is_almost_equal(particle.weight(), PROB_THRESHOLD):
                        self.map_counter -= 1
                    elif self.map_counter <= 0:
                        self.map_counter = 1
                    else:
                        self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)

                    if self.map_counter < MAX_BAD_COUNT:
                        self.first_locate = True
                        self.map_counter = 0
                        self.get_logger().info("Restart localization")
            else:
                self.get_logger().warning("CANNOT FIND ANY FEATURES !!!!!")

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_img = curr_img
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self._broadcast_transform()

    def state_callback(self, msg: Odometry) -> None:
        self.z = msg.pose.pose.position.z
        self.angle_x = msg.twist.twist.angular.x
        self.angle_y = msg.twist.twist.angular.y

    def reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.locate_position = True
        self.first_locate = True
        self.map_counter = 0
        self.max_map_counter = 0
        response.success = True
        response.message = "Localization reset"
        return response

    def _update_pose_msg(self, x: float, y: float, z: float, yaw: float) -> None:
        self.posemsg.pose.position.x = x
        self.posemsg.pose.position.y = y
        self.posemsg.pose.position.z = z
        self.posemsg.pose.orientation.z = sin(yaw / 2.0)
        self.posemsg.pose.orientation.w = cos(yaw / 2.0)

    def _broadcast_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base"
        transform.transform.translation.x = self.pos[0]
        transform.transform.translation.y = self.pos[1]
        transform.transform.translation.z = self.z
        transform.transform.rotation.z = sin(self.pos[3] / 2.0)
        transform.transform.rotation.w = cos(self.pos[3] / 2.0)
        self.tf_broadcaster.sendTransform(transform)


def is_almost_equal(x, y):
    epsilon = 1 * 10 ** (-8)
    return abs(x - y) <= epsilon


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()