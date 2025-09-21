import json
import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        self.declare_parameter('k', 27.0)
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('trim', 0.0)
        self.declare_parameter('limit', 1.0)
        self.declare_parameter('baseline', 0.1)
        self.declare_parameter('radius', 0.0318)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('omega_max', 8.0)

        self.k = float(self.get_parameter('k').value)
        self.gain = float(self.get_parameter('gain').value)
        self.trim = float(self.get_parameter('trim').value)
        self.limit = float(self.get_parameter('limit').value)
        self.baseline = float(self.get_parameter('baseline').value)
        self.radius = float(self.get_parameter('radius').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.omega_max = float(self.get_parameter('omega_max').value)

        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, 'wheels_cmd', 10)
        self.pub_velocity = self.create_publisher(Twist2DStamped, 'velocity', 10)

        self.create_subscription(Twist2DStamped, 'car_cmd', self._on_car_cmd, 10)
        self.create_subscription(WheelsCmdStamped, 'wheels_cmd_executed', self._on_wheels_cmd_executed, 10)

        self.get_logger().info('Initialized with: %s' % json.dumps({
            'gain': self.gain,
            'trim': self.trim,
            'baseline': self.baseline,
            'radius': self.radius,
            'k': self.k,
            'limit': self.limit,
            'v_max': self.v_max,
            'omega_max': self.omega_max,
        }, sort_keys=True))

    def _on_car_cmd(self, msg: Twist2DStamped):
        v = max(min(msg.v, self.v_max), -self.v_max)
        omega = max(min(msg.omega, self.omega_max), -self.omega_max)

        k_r = k_l = self.k
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = (v + 0.5 * omega * self.baseline) / self.radius
        omega_l = (v - 0.5 * omega * self.baseline) / self.radius

        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        out = WheelsCmdStamped()
        out.header = msg.header
        out.vel_right = u_r_limited
        out.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(out)

    def _on_wheels_cmd_executed(self, msg: WheelsCmdStamped):
        k_r = k_l = self.k
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = msg.vel_right / k_r_inv
        omega_l = msg.vel_left / k_l_inv

        v = (self.radius * omega_r + self.radius * omega_l) / 2.0
        omega = (self.radius * omega_r - self.radius * omega_l) / self.baseline

        vel = Twist2DStamped()
        vel.header = msg.header
        vel.v = v
        vel.omega = omega
        self.pub_velocity.publish(vel)


def main():
    rclpy.init()
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


