import time
from typing import List
import os
import yaml

import rclpy
from rclpy.node import Node

from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool


class LEDEmitterNode(Node):
    def __init__(self) -> None:
        super().__init__('led_emitter_node')

        # Declare and get parameters
        self.declare_parameter('robot_type', 'duckiebot')
        self.declare_parameter('LED_scale', 0.8)
        self.declare_parameter('protocol_file', '')

        self.robot_type: str = self.get_parameter('robot_type').get_parameter_value().string_value
        self._LED_scale: float = self.get_parameter('LED_scale').get_parameter_value().double_value
        protocol_file: str = self.get_parameter('protocol_file').get_parameter_value().string_value

        if not protocol_file or not os.path.exists(protocol_file):
            raise RuntimeError(f"LED protocol file not found: {protocol_file}")

        # TODO: this should be passed as a ros parameter
        with open(protocol_file, 'r') as f:
            cfg = yaml.safe_load(f) or {}

        self._LED_protocol = cfg.get('LED_protocol', {})
        self._channel_order = cfg.get('channel_order', {'duckiebot': 'RGB', 'traffic_light': 'GRB'})

        # Initialize LEDs to be off
        self.LEDspattern: List[List[float]] = [[0.0, 0.0, 0.0]] * 5
        self.pattern: List[List[float]] = [[0.0, 0.0, 0.0]] * 5
        self.frequency_mask: List[int] = [0] * 5
        self.current_pattern_name: str = 'LIGHT_OFF'

        # Publisher
        self.pub_leds = self.create_publisher(LEDPattern, 'led_pattern', 1)

        # Services
        self.srv_set_custom = self.create_service(SetCustomLEDPattern, 'set_custom_pattern', self.srvSetCustomLEDPattern)
        self.srv_set_pattern = self.create_service(ChangePattern, 'set_pattern', self.srvSetPattern)
        self.srv_switch = self.create_service(SetBool, 'switch', self.srvSwitch)

        # Scale intensity of the LEDs
        colors = self._LED_protocol.get('colors', {})
        for name, triplet in colors.items():
            for i in range(3):
                triplet[i] = triplet[i] * self._LED_scale

        # Remap colors if robot does not have an RGB ordering
        try:
            if self._channel_order.get(self.robot_type, 'RGB') != 'RGB':
                protocol = self._LED_protocol
                remapped = {}
                for name, col in protocol.get('colors', {}).items():
                    remapped[name] = self.remapColors(col)
                protocol['colors'] = remapped
                self._LED_protocol = protocol
        except Exception as e:
            self.get_logger().warn(f"Channel remap failed: {e}")

        # Initialize pattern and timer
        self.timer = None
        self.is_on = False
        self.frequency = 0.0
        self.changePattern('WHITE')

        self.get_logger().info('LEDEmitterNode initialized')

    def srvSetCustomLEDPattern(self, req: SetCustomLEDPattern.Request, res: SetCustomLEDPattern.Response):
        protocol = self._LED_protocol
        protocol.setdefault('signals', {})['custom'] = {
            'color_mask': list(req.pattern.color_mask),
            'color_list': list(req.pattern.color_list),
            'frequency_mask': list(req.pattern.frequency_mask),
            'frequency': req.pattern.frequency,
        }
        self._LED_protocol = protocol
        self.changePattern('custom')
        return res

    def srvSwitch(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Switch service handler for FSM compatibility.
        
        When enabled (True), sets LEDs to WHITE.
        When disabled (False), sets LEDs to LIGHT_OFF.
        """
        if req.data:
            self.changePattern('WHITE')
            self.get_logger().info('LED emitter enabled - set to WHITE')
        else:
            self.changePattern('LIGHT_OFF')
            self.get_logger().info('LED emitter disabled - set to LIGHT_OFF')
        
        res.success = True  # Always return success
        return res

    def _cycle_timer(self) -> None:
        self.updateLEDs()

    def updateLEDs(self) -> None:
        if not self.frequency:
            for i in range(5):
                self.LEDspattern[i] = self.pattern[i]
        else:
            if self.is_on:
                for i in range(5):
                    if i < len(self.frequency_mask) and self.frequency_mask[i]:
                        self.LEDspattern[i] = [0.0, 0.0, 0.0]
                self.is_on = False
            else:
                for i in range(5):
                    self.LEDspattern[i] = self.pattern[i]
                self.is_on = True

        self.publishLEDs()

    def publishLEDs(self) -> None:
        msg = LEDPattern()
        for i in range(5):
            rgba = ColorRGBA()
            rgba.r = float(self.LEDspattern[i][0])
            rgba.g = float(self.LEDspattern[i][1])
            rgba.b = float(self.LEDspattern[i][2])
            rgba.a = 1.0
            msg.rgb_vals.append(rgba)
        self.pub_leds.publish(msg)

    def srvSetPattern(self, req: ChangePattern.Request, res: ChangePattern.Response):
        self.changePattern(str(req.pattern_name.data))
        return res

    def changePattern(self, pattern_name: str) -> None:
        if not pattern_name:
            return
        if self.current_pattern_name == pattern_name and pattern_name != 'custom':
            return
        signals = self._LED_protocol.get('signals', {})
        if pattern_name.strip("'\"") not in signals:
            self.get_logger().error(f"Pattern name {pattern_name} not found in the list of patterns.")
            return

        self.current_pattern_name = pattern_name

        color_list = signals[pattern_name].get('color_list')
        colors = self._LED_protocol.get('colors', {})

        if isinstance(color_list, str):
            self.pattern = [colors.get(color_list, [0, 0, 0])] * 5
        else:
            if len(color_list) != 5:
                self.get_logger().error('The color list should be a string or a list of length 5.')
                return
            self.pattern = [[0.0, 0.0, 0.0]] * 5
            for i in range(5):
                if isinstance(color_list[i], str):
                    self.pattern[i] = colors.get(color_list[i], [0, 0, 0])
                elif isinstance(color_list[i], list) and len(color_list[i]) == 3:
                    self.pattern[i] = [max(0.0, min(float(c), 255.0)) for c in color_list[i]]
                else:
                    self.get_logger().error('LEDs color passed as RGB must be lists of 3 values [0,255].')
                    return

        # Normalize frequency_mask to length 5
        fm = list(signals[pattern_name].get('frequency_mask', []))
        if len(fm) < 5:
            fm = fm + [0] * (5 - len(fm))
        elif len(fm) > 5:
            fm = fm[:5]
        self.frequency_mask = fm
        self.frequency = signals[pattern_name].get('frequency', 0.0)

        if self.frequency == 0:
            self.updateLEDs()
        self.changeFrequency()
        self.get_logger().info(f'Pattern changed to ({pattern_name}), cycle: {self.frequency}')

    def changeFrequency(self) -> None:
        if self.frequency == 0:
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()
                self.timer = None
            return
        try:
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()
            d = 1.0 / (2.0 * self.frequency)
            self.timer = self.create_timer(d, self._cycle_timer)
        except Exception:
            self.frequency = None
            self.current_pattern_name = None

    def remapColors(self, color: List[float]) -> List[float]:
        allowed_orderings = ['RGB', 'RBG', 'GBR', 'GRB', 'BGR', 'BRG']
        requested_ordering = self._channel_order.get(self.robot_type, 'RGB')
        if requested_ordering not in allowed_orderings:
            self.get_logger().warn(f'The current channel order {requested_ordering} is not supported.')
            return color
        reordered_triplet: List[float] = []
        rgb_map = {'R': 0, 'G': 1, 'B': 2}
        for channel_color in requested_ordering:
            reordered_triplet.append(color[rgb_map[channel_color]])
        return reordered_triplet


def main() -> None:
    rclpy.init()
    node = LEDEmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Turn LEDs off on shutdown
        try:
            node.changePattern('LIGHT_OFF')
            time.sleep(1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
