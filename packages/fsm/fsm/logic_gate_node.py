#!/usr/bin/env python3
import yaml
from typing import Dict, Any, List

import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import AprilTagsWithInfos, BoolStamped, Twist2DStamped
from std_msgs.msg import Float32, UInt8

classes = {
    "BoolStamped": BoolStamped,
    "Twist2DStamped": Twist2DStamped,
    "AprilTagsWithInfos": AprilTagsWithInfos,
    "Float32": Float32,
    "UInt8": UInt8,
}


class LogicGateNode(Node):
    def __init__(self):
        super().__init__('logic_gate_node')
        
        # Enable this for printing all logic gate operations
        self.debugging = False

        self.node_name = self.get_name()
        
        # Declare parameters
        self.declare_parameter('gates', '{}')
        self.declare_parameter('events', '{}')
        
        # Load and parse gates
        try:
            gates_yaml = self.get_parameter('gates').get_parameter_value().string_value
            self.gates_dict = yaml.safe_load(gates_yaml) if gates_yaml else {}
        except Exception:
            self.gates_dict = {}
            
        # validate gate
        if not self._validateGates(self.gates_dict):
            self.get_logger().error("Invalid gate_type.")
            return

        # Load and parse events
        try:
            events_yaml = self.get_parameter('events').get_parameter_value().string_value
            self.events_dict = yaml.safe_load(events_yaml) if events_yaml else {}
        except Exception:
            self.events_dict = {}
            
        if not self._validateEvents():
            self.get_logger().error("Invalid event definition.")
            return

        self.sub_list = list()
        self.pub_dict = dict()
        self.event_msg_dict = dict()
        self.event_trigger_dict = dict()
        self.last_published_msg = None
        
        for gate_name, gate_dict in list(self.gates_dict.items()):
            output_topic_name = gate_dict["output_topic"]
            self.pub_dict[gate_name] = self.create_publisher(BoolStamped, output_topic_name, 1)
            
        for event_name, event_dict in list(self.events_dict.items()):
            topic_name = event_dict["topic"]
            self.event_trigger_dict[event_name] = event_dict["trigger"]
            # Initialize local copy as None
            self.event_msg_dict[event_name] = None
            msg_type = classes[event_dict["msg_type"]]
            self.sub_list.append(
                self.create_subscription(
                    msg_type, topic_name, 
                    lambda msg, event=event_name: self.cbBoolStamped(msg, event), 1)
            )

    def _validateEvents(self) -> bool:
        valid_flag = True
        for event_name, event_dict in list(self.events_dict.items()):
            if "topic" not in event_dict:
                self.get_logger().fatal(f"[{self.node_name}] topic not defined for event {event_name}")
                valid_flag = False
        return valid_flag

    def _validateGates(self, gates_dict: Dict[str, Any]) -> bool:
        valid_gate_types = ["AND", "OR"]
        for gate_name, gate_dict in list(gates_dict.items()):
            gate_type = gate_dict["gate_type"]
            if gate_type not in valid_gate_types:
                self.get_logger().fatal(f"[{self.node_name}] gate_type {gate_type} is not valid.")
                return False
        return True

    def publish(self, msg, gate_name: str):
        if msg is None:
            return
        self.pub_dict[gate_name].publish(msg)

    def getOutputMsg(self, gate_name: str, inputs: List[str]):
        bool_list = list()
        latest_time_stamp = rclpy.time.Time().to_msg()

        for event_name, event_msg in list(self.event_msg_dict.items()):
            if event_name in inputs:  # one of the inputs to gate

                if self.debugging:
                    print(("sub-event: " + event_name))

                if event_msg is None:
                    if "default" in self.events_dict[event_name]:
                        bool_list.append(self.events_dict[event_name]["default"])
                    else:
                        bool_list.append(False)
                else:
                    if "field" in self.events_dict[event_name]:  # if special type of message
                        if (
                            getattr(event_msg, self.events_dict[event_name]["field"])
                            == self.event_trigger_dict[event_name]
                        ):
                            bool_list.append(True)
                        else:
                            bool_list.append(False)
                    else:  # else BoolStamped
                        if event_msg.data == self.event_trigger_dict[event_name]:
                            bool_list.append(True)
                        else:
                            bool_list.append(False)
                    # Keeps track of latest timestamp
                    if hasattr(event_msg, 'header') and hasattr(event_msg.header, 'stamp'):
                        msg_time = rclpy.time.Time.from_msg(event_msg.header.stamp)
                        if msg_time > rclpy.time.Time.from_msg(latest_time_stamp):
                            latest_time_stamp = event_msg.header.stamp

        # Perform logic operation
        msg = BoolStamped()
        msg.header.stamp = latest_time_stamp

        gate = self.gates_dict.get(gate_name)
        gate_type = gate.get("gate_type")
        if gate_type == "AND":
            msg.data = all(bool_list)
        elif gate_type == "OR":
            msg.data = any(bool_list)

        if self.debugging:
            print((bool_list, "->", msg.data))

        return msg

    def cbBoolStamped(self, msg, event_name: str):
        self.event_msg_dict[event_name] = msg

        for gate_name, gate_dict in list(self.gates_dict.items()):
            inputs = gate_dict.get("inputs")
            if event_name in inputs:
                self.publish(self.getOutputMsg(gate_name, inputs), gate_name)

    def on_shutdown(self):
        self.get_logger().info(f"[{self.node_name}] Shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = LogicGateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
