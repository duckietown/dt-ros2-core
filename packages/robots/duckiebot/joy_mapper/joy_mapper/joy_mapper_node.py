#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy


# Button List index of joy.buttons array:
#   0: A
#   1: B
#   2: X
#   3: Y
#   4: Left Back
#   5: Right Back
#   6: Back
#   7: Start
#   8: Logitek
#   9: Left joystick
#   10: Right joystick


class JoyMapperNode(Node):
    """Interprets the Joystick commands (ROS 2).

    The `JoyMapperNode` receives :obj:`Joy` messages from a physical joystick or a virtual one,
    interprets the button presses and acts accordingly.

    **Joystick bindings:**

    +----------------------+------------------+------------------------------------------------+
    | Physical joystick    | Virtual joystick | Action                                         |
    +======================+==================+================================================+
    | Directional controls | Arrow keys       | Move the Duckiebot (if not in lane-following)  |
    +----------------------+------------------+------------------------------------------------+
    | Start button         | `A` key          | Start lane-following                           |
    +----------------------+------------------+------------------------------------------------+
    | Back button          | `S` key          | Stop lane-following                            |
    +----------------------+------------------+------------------------------------------------+
    | Y button             | `E` key          | Toggle Emergency Stop                          |
    +----------------------+------------------+------------------------------------------------+

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        speed_gain (:obj:`float`): Gain for the directional joystick keys (forward/reverse)
        steer_gain (:obj:`float`): Gain for the directional joystick keys (steering angle)
        bicycle_kinematics (:obj:`bool`): `True` for bicycle kinematics; `False` for holonomic
           kinematics. Default is `False`
        simulated_vehicle_length (:obj:`float`): Used in bicycle kinematics model

    Subscriber:
        joy (:obj:`Joy`): The command read from joystick
        emergency_stop (:obj:`BoolStamped`): The emergency stop status

    Publishers:
        car_cmd (:obj:`duckietown_msgs/Twist2DStamped`): Wheels command for Duckiebot, based
           on the directional buttons pressed
        joystick_override (:obj:`duckietown_msgs/BoolStamped`): Boolean that is used to control
           whether lane-following or joystick control is on
        emergency_stop (:obj:`duckietown_msgs/BoolStamped`): Emergency stop signal
    """

    def __init__(self, node_name='joy_mapper'):
        # Initialize the ROS 2 node
        super().__init__(node_name)

        # emergency stop disabled by default
        self.e_stop = False

        # Declare and get parameters
        self.declare_parameter('speed_gain', 0.41)
        self.declare_parameter('steer_gain', 8.3)
        self.declare_parameter('bicycle_kinematics', False)
        self.declare_parameter('simulated_vehicle_length', 0.18)

        self._speed_gain = self.get_parameter('speed_gain').value
        self._steer_gain = self.get_parameter('steer_gain').value
        self._bicycle_kinematics = self.get_parameter('bicycle_kinematics').value
        self._simulated_vehicle_length = self.get_parameter('simulated_vehicle_length').value

        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers (use relative topic names for proper namespacing)
        self.pub_car_cmd = self.create_publisher(
            Twist2DStamped, 'car_cmd', qos_cmd
        )
        self.pub_joy_override = self.create_publisher(
            BoolStamped, 'joystick_override', qos_cmd
        )
        self.pub_e_stop = self.create_publisher(
            BoolStamped, 'emergency_stop', qos_cmd
        )

        # Subscribers (use relative topic names for proper namespacing)
        self.sub_joy = self.create_subscription(
            Joy, 'joy', self.joy_cb, qos_sensor
        )
        self.sub_e_stop = self.create_subscription(
            BoolStamped, 'emergency_stop', self.estop_cb, qos_cmd
        )

        self.get_logger().info(f'{node_name} initialized with speed_gain={self._speed_gain}, steer_gain={self._steer_gain}')

    def estop_cb(self, estop_msg):
        """
        Callback that processes the received :obj:`BoolStamped` messages.

        Args:
            estop_msg (:obj:`BoolStamped`): the emergency_stop message to process.
        """
        self.e_stop = estop_msg.data

    def joy_cb(self, joy_msg):
        """
        Callback that processes the received :obj:`Joy` messages.

        Args:
            joy_msg (:obj:`Joy`): the joystick message to process.
        """
        # Navigation buttons
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        # Left stick V-axis. Up is positive
        car_cmd_msg.v = joy_msg.axes[1] * self._speed_gain
        if self._bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = joy_msg.axes[3] * self._steer_gain
            car_cmd_msg.omega = car_cmd_msg.v / self._simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = joy_msg.axes[3] * self._steer_gain
        self.pub_car_cmd.publish(car_cmd_msg)

        # Back button: Stop LF
        if joy_msg.buttons[6] == 1:
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = True
            self.get_logger().info("Joystick override = True (Back button pressed)")
            self.pub_joy_override.publish(override_msg)

        # Start button: Start LF
        elif joy_msg.buttons[7] == 1:
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = False
            self.get_logger().info("Joystick override = False (Start button pressed)")
            self.pub_joy_override.publish(override_msg)

        # Y button: Emergency Stop
        elif joy_msg.buttons[3] == 1:
            self.e_stop = not self.e_stop
            estop_msg = BoolStamped()
            estop_msg.header.stamp = joy_msg.header.stamp
            estop_msg.data = self.e_stop
            self.pub_e_stop.publish(estop_msg)
            self.get_logger().info(f"Emergency stop toggled to {self.e_stop}")

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                self.get_logger().warn(f"No binding for joy_msg.buttons = {joy_msg.buttons}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyMapperNode('joy_mapper')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
