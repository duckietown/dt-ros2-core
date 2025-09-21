import math
import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped


class VelocityToPoseNode(Node):
    def __init__(self):
        super().__init__('velocity_to_pose_node')
        self.veh_name = self.get_namespace().strip('/')
        self.last_pose = Pose2DStamped()
        self.last_theta_dot = 0.0
        self.last_v = 0.0

        self.pub_pose = self.create_publisher(Pose2DStamped, 'pose', 10)
        self.create_subscription(Twist2DStamped, 'velocity', self._on_velocity, 10)
        self.get_logger().info('Initialized.')

    def _on_velocity(self, msg: Twist2DStamped):
        if self.last_pose.header.stamp.sec > 0 or self.last_pose.header.stamp.nanosec > 0:
            dt = (msg.header.stamp.sec - self.last_pose.header.stamp.sec) + \
                 (msg.header.stamp.nanosec - self.last_pose.header.stamp.nanosec) / 1e9

            theta_delta = self.last_theta_dot * dt
            if abs(self.last_theta_dot) < 1e-6:
                x_delta = self.last_v * dt
                y_delta = 0.0
            else:
                radius = self.last_v / self.last_theta_dot
                x_delta = radius * math.sin(theta_delta)
                y_delta = radius * (1.0 - math.cos(theta_delta))

            theta_res = self.last_pose.theta + theta_delta
            x_res = self.last_pose.x + x_delta * math.cos(self.last_pose.theta) - y_delta * math.sin(self.last_pose.theta)
            y_res = self.last_pose.y + y_delta * math.cos(self.last_pose.theta) + x_delta * math.sin(self.last_pose.theta)

            self.last_pose.theta = theta_res
            self.last_pose.x = x_res
            self.last_pose.y = y_res

            out = Pose2DStamped()
            out.header = msg.header
            out.header.frame_id = self.veh_name
            out.theta = theta_res
            out.x = x_res
            out.y = y_res
            self.pub_pose.publish(out)

        self.last_pose.header.stamp = msg.header.stamp
        self.last_theta_dot = msg.omega
        self.last_v = msg.v


def main():
    rclpy.init()
    node = VelocityToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


