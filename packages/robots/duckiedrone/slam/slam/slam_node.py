"""ROS 2 FastSLAM node for Duckiedrone."""

from math import cos, sin
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

from .slam_helper.helper import FastSLAM

CAMERA_WIDTH = 480
CAMERA_HEIGHT = 640
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2.0, (CAMERA_HEIGHT - 1) / 2.0]).reshape(-1, 1, 2)
NUM_PARTICLE = 20
NUM_FEATURES = 50


class SLAMNode(Node):
    """Run FastSLAM on Duckiedrone camera features."""

    def __init__(self) -> None:
        super().__init__("slam_node")
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(CompressedImage, "~/image", self.image_callback, 1)
        self.create_subscription(Odometry, "~/state", self.state_callback, 1)
        self.posepub = self.create_publisher(PoseStamped, "~/pose", 1)
        self.create_service(Trigger, "~/reset_transform", self.reset_callback)

        self.detector = cv2.ORB.create(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
        self.estimator = FastSLAM()
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
        self.alpha_yaw = 0.1
        self.hybrid_alpha = 0.3

    def image_callback(self, msg: CompressedImage) -> None:
        try:
            curr_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as error:
            self.get_logger().error(f"Failed to convert image: {error}")
            return

        self.posemsg.header.stamp = self.get_clock().now().to_msg()
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)
            if curr_kp is not None and len(curr_kp) > 0:
                if self.first_locate:
                    pose = self.estimator.generate_particles(NUM_PARTICLE)
                    self.first_locate = False
                    self.pos = pose
                    self.update_pose(pose)
                else:
                    pose, weight = self.estimator.run(self.z, self.prev_kp, self.prev_des, curr_kp, curr_des)
                    self.update_position(pose)
                    self.update_pose(self.pos)
                    self.get_logger().info(f"--Weight: {weight:f}")
            else:
                self.get_logger().warning("Cannot find any features")

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_img = curr_img
        self._broadcast_transform()

    def state_callback(self, msg: Odometry) -> None:
        self.z = msg.pose.pose.position.z
        self.angle_x = msg.twist.twist.angular.x
        self.angle_y = msg.twist.twist.angular.y

    def reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.locate_position = True
        self.first_locate = True
        response.success = True
        response.message = "SLAM reset"
        return response

    def update_position(self, new_pose) -> None:
        self.pos = [
            self.hybrid_alpha * new_pose[0] + (1.0 - self.hybrid_alpha) * self.pos[0],
            self.hybrid_alpha * new_pose[1] + (1.0 - self.hybrid_alpha) * self.pos[1],
            self.z,
            self.alpha_yaw * new_pose[3] + (1.0 - self.alpha_yaw) * self.pos[3],
        ]

    def update_pose(self, pose) -> None:
        self.posemsg.pose.position.x = pose[0]
        self.posemsg.pose.position.y = pose[1]
        self.posemsg.pose.position.z = self.z
        self.posemsg.pose.orientation.z = sin(pose[3] / 2.0)
        self.posemsg.pose.orientation.w = cos(pose[3] / 2.0)
        self.posepub.publish(self.posemsg)

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


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()