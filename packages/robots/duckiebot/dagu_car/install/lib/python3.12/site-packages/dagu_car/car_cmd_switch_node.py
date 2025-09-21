import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import Twist2DStamped, FSMState


class CarCmdSwitchNode(Node):
    def __init__(self):
        super().__init__('car_cmd_switch_node')

        # ROS 2 parameters do not support dict types. Use in-code defaults.
        self.declare_parameter('mode_topic', 'fsm_node/mode')

        self.mappings = {
            'DETECT_INTERSECTION_TYPE': 'stop',
            'INTERSECTION_CONTROL': 'intersection',
            'LANE_FOLLOWING': 'lane',
            'NORMAL_JOYSTICK_CONTROL': 'joystick',
            'STOP': 'stop',
            'STOP_SIGN_INTERSECTION': 'stop',
            'TRAFFIC_LIGHT_INTERSECTION': 'stop',
        }

        self.mode_topic = str(self.get_parameter('mode_topic').value)

        self.source_topics = {
            'coordination': 'coordinator_node/car_cmd',
            'intersection': 'unicorn_intersection_node/car_cmd',
            'joystick': 'joy_mapper_node/car_cmd',
            'lane': 'lane_controller_node/car_cmd',
            'stop': 'simple_stop_controller_node/car_cmd',
        }

        self.current_src_name = 'joystick'

        self.pub_cmd = self.create_publisher(Twist2DStamped, 'cmd', 10)
        self.create_subscription(FSMState, self.mode_topic, self._on_fsm_state, 10)
        self.cmd_subscriptions = {}
        for src_name, topic in self.source_topics.items():
            self.cmd_subscriptions[src_name] = self.create_subscription(
                Twist2DStamped, topic, lambda msg, s=src_name: self._on_cmd(msg, s), 10
            )

        self.get_logger().info('Initialized.')

    def _on_fsm_state(self, msg: FSMState):
        src = self.mappings.get(msg.state)
        self.current_src_name = src if src is not None else self.current_src_name

    def _on_cmd(self, msg: Twist2DStamped, src_name: str):
        if src_name == self.current_src_name:
            self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = CarCmdSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


