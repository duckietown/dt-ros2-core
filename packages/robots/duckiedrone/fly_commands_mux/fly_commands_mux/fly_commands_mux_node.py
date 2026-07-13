"""ROS 2 fly commands multiplexer for Duckiedrone."""

from typing import Optional

import rclpy
from duckietown_msgs.msg import DroneControl
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node


class FlyCommandsMuxNode(Node):
    """Combine manual and autonomous drone commands into RC overrides."""

    def __init__(self) -> None:
        super().__init__("fly_commands_mux_node")

        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("roll_override", True)
        self.declare_parameter("pitch_override", True)
        self.declare_parameter("yaw_override", True)
        self.declare_parameter("throttle_override", True)
        self.declare_parameter("command_ttl", 0.5)

        self.manual_commands: Optional[DroneControl] = None
        self.autonomous_commands: Optional[DroneControl] = None
        now = self.get_clock().now()
        self.last_stamp_manual = now
        self.last_stamp_autonomous = now

        self.create_subscription(DroneControl, "~/commands/manual", self.cb_manual, 1)
        self.create_subscription(DroneControl, "~/commands/autonomous", self.cb_autonomous, 1)
        self.pub_cmds = self.create_publisher(OverrideRCIn, "~/commands/output", 1)

        frequency = float(self.get_parameter("publish_frequency").value)
        frequency = max(frequency, 1e-3)
        self._timer = self.create_timer(1.0 / frequency, self.publish_fly_commands)

    def cb_manual(self, msg: DroneControl) -> None:
        """Store the latest manual control command."""
        self.last_stamp_manual = self.get_clock().now()
        self.manual_commands = msg

    def cb_autonomous(self, msg: DroneControl) -> None:
        """Store the latest autonomous control command."""
        self.last_stamp_autonomous = self.get_clock().now()
        self.autonomous_commands = msg

    def publish_fly_commands(self) -> None:
        """Publish the latest valid muxed command as RC channel overrides."""
        command_ttl = float(self.get_parameter("command_ttl").value)
        now_seconds = self.get_clock().now().nanoseconds / 1e9

        if now_seconds - (self.last_stamp_manual.nanoseconds / 1e9) > command_ttl:
            self.manual_commands = None
        if now_seconds - (self.last_stamp_autonomous.nanoseconds / 1e9) > command_ttl:
            self.autonomous_commands = None

        if self.manual_commands is None and self.autonomous_commands is None:
            return

        if self.manual_commands is None:
            self.pub_cmds.publish(self._to_override_msg(self.autonomous_commands))
            return

        if self.autonomous_commands is None:
            self.pub_cmds.publish(self._to_override_msg(self.manual_commands))
            return

        roll_override = bool(self.get_parameter("roll_override").value)
        pitch_override = bool(self.get_parameter("pitch_override").value)
        yaw_override = bool(self.get_parameter("yaw_override").value)
        throttle_override = bool(self.get_parameter("throttle_override").value)

        selected = DroneControl()
        selected.roll = self.manual_commands.roll if roll_override else self.autonomous_commands.roll
        selected.pitch = self.manual_commands.pitch if pitch_override else self.autonomous_commands.pitch
        selected.yaw = self.manual_commands.yaw if yaw_override else self.autonomous_commands.yaw
        selected.throttle = (
            self.manual_commands.throttle
            if throttle_override
            else self.autonomous_commands.throttle
        )
        self.pub_cmds.publish(self._to_override_msg(selected))

    @staticmethod
    def _to_override_msg(command: Optional[DroneControl]) -> OverrideRCIn:
        """Convert a DroneControl command to the flight-controller override format."""
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        if command is None:
            return msg

        msg.channels[0] = int(command.roll)
        msg.channels[1] = int(command.pitch)
        msg.channels[2] = int(command.throttle)
        msg.channels[3] = int(command.yaw)
        return msg


def main(args: Optional[list[str]] = None) -> None:
    """Run the fly commands mux node."""
    rclpy.init(args=args)
    node = FlyCommandsMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()