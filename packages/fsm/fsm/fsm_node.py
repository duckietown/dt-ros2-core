#!/usr/bin/env python3
import copy
import yaml
from typing import Dict, List, Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from duckietown_msgs.msg import BoolStamped, FSMState
from duckietown_msgs.srv import SetFSMState, ChangePattern
from std_srvs.srv import SetBool
from std_msgs.msg import String


class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')
        self.node_name = self.get_name()

        # Declare all parameters first
        self.declare_parameter('initial_state', '')
        
        # Declare nested parameters for complex structures as strings
        self.declare_parameter('states', '{}')
        self.declare_parameter('global_transitions', '{}')
        self.declare_parameter('nodes', '{}')
        self.declare_parameter('events', '{}')

        # Load and parse states
        try:
            states_yaml = self.get_parameter('states').get_parameter_value().string_value
            self.states_dict = yaml.safe_load(states_yaml) if states_yaml else {}
        except Exception:
            self.states_dict = {}

        # Validate state and global transitions
        if not self._validateStates(self.states_dict):
            self.get_logger().error(f"[{self.node_name}] Incoherent definition.")
            return

        # Load global transitions
        try:
            global_transitions_yaml = self.get_parameter('global_transitions').get_parameter_value().string_value
            self.global_transitions_dict = yaml.safe_load(global_transitions_yaml) if global_transitions_yaml else {}
        except Exception:
            self.global_transitions_dict = {}
        
        if not self._validateGlobalTransitions(self.global_transitions_dict, list(self.states_dict.keys())):
            self.get_logger().error(f"[{self.node_name}] Incoherent definition.")
            return

        # Setup initial state
        self.state_msg = FSMState()
        self.state_msg.state = self.get_parameter('initial_state').get_parameter_value().string_value
        print("Initial state:", self.state_msg.state)
        self.state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Setup publisher and publish initial state
        self.pub_state = self.create_publisher(FSMState, '~/mode', 1)

        # Provide service
        self.srv_state = self.create_service(SetFSMState, '~/set_state', self.cbSrvSetState)

        # Construct service calls
        self.srv_dict = dict()
        try:
            nodes_yaml = self.get_parameter('nodes').get_parameter_value().string_value
            nodes = yaml.safe_load(nodes_yaml) if nodes_yaml else {}
        except Exception:
            nodes = {}
            
        self.active_nodes = None

        for node_name, service_name in list(nodes.items()):
            self.get_logger().info(f"FSM creating client for service {service_name}")
            try:
                self.srv_dict[node_name] = self.create_client(SetBool, service_name)
                self.get_logger().info(f"FSM created client for service {service_name}")
            except Exception as e:
                self.get_logger().warn(f"{e}")

        # to change the LEDs
        self.changePattern = self.create_client(ChangePattern, '~/set_pattern')

        # Process events definition
        try:
            events_yaml = self.get_parameter('events').get_parameter_value().string_value
            param_events_dict = yaml.safe_load(events_yaml) if events_yaml else {}
        except Exception:
            param_events_dict = {}
        
        # Validate events definition
        if not self._validateEvents(param_events_dict):
            self.get_logger().error(f"[{self.node_name}] Invalid event definition.")
            return

        self.sub_list = list()
        self.event_trigger_dict = dict()
        for event_name, event_dict in list(param_events_dict.items()):
            topic_name = event_dict["topic"]
            msg_type = event_dict["msg_type"]
            self.event_trigger_dict[event_name] = event_dict["trigger"]
            # TODO so far I can't figure out how to put msg_type instead of BoolStamped.
            # importlib might help. But it might get too complicated since different type
            self.sub_list.append(
                self.create_subscription(BoolStamped, topic_name, 
                                       lambda msg, event=event_name: self.cbEvent(msg, event), 1)
            )

        self.get_logger().info(f"[{self.node_name}] Initialized.")
        # Publish initial state
        self.publish()

    def _validateGlobalTransitions(self, global_transitions: Dict[str, str], valid_states: List[str]) -> bool:
        pass_flag = True
        for event_name, state_name in list(global_transitions.items()):
            if state_name not in valid_states:
                self.get_logger().error(
                    f"[{self.node_name}] State {state_name} is not valid. (From global_transitions of "
                    f"{event_name})"
                )
                pass_flag = False
        return pass_flag

    def _validateEvents(self, events_dict: Dict[str, Any]) -> bool:
        pass_flag = True
        for event_name, event_dict in list(events_dict.items()):
            if "topic" not in event_dict:
                self.get_logger().error(f"[{self.node_name}] Event {event_name} missing topic definition.")
                pass_flag = False
            if "msg_type" not in event_dict:
                self.get_logger().error(f"[{self.node_name}] Event {event_name} missing msg_type definition.")
                pass_flag = False
            if "trigger" not in event_dict:
                self.get_logger().error(f"[{self.node_name}] Event {event_name} missing trigger definition.")
                pass_flag = False
        return pass_flag

    def _validateStates(self, states_dict: Dict[str, Any]) -> bool:
        pass_flag = True
        valid_states = list(states_dict.keys())
        for state, state_dict in list(states_dict.items()):
            # Validate the existence of all reachable states
            transitions_dict = state_dict.get("transitions")
            if transitions_dict is None:
                continue
            else:
                for transition, next_state in list(transitions_dict.items()):
                    if next_state not in valid_states:
                        self.get_logger().error(
                            f"[{self.node_name}] {next_state} not a valide state. (From {state} with event "
                            f"{transition})"
                        )
                        pass_flag = False
        return pass_flag

    def _getNextState(self, state_name: str, event_name: str) -> str:
        if not self.isValidState(state_name):
            self.get_logger().warn(f"[{self.node_name}] {state_name} not defined. Treat as terminal. ")
            return None

        # state transitions overwrites global transition
        state_dict = self.states_dict.get(state_name)
        if "transitions" in state_dict:
            next_state = state_dict["transitions"].get(event_name)
        else:
            next_state = None

        # state transitions overwrites global transitions
        if next_state is None:
            # No state transition defined, look up global transition
            next_state = self.global_transitions_dict.get(event_name)  # None when no global transitions
        return next_state

    def _getActiveNodesOfState(self, state_name: str) -> List[str]:
        state_dict = self.states_dict[state_name]
        active_nodes = state_dict.get("active_nodes")
        if active_nodes is None:
            self.get_logger().warn(f"[{self.node_name}] No active nodes defined for {state_name}. Deactive all nodes.")
            active_nodes = []
        return active_nodes

    def _getLightsofState(self, state_name: str) -> str:
        state_dict = self.states_dict[state_name]
        lights = state_dict.get("lights")
        return lights

    def publish(self):
        self.publishBools()
        self.publishState()
        self.updateLights()

    def isValidState(self, state: str) -> bool:
        return state in list(self.states_dict.keys())

    def cbSrvSetState(self, request, response):
        if self.isValidState(request.state):
            self.state_msg.header.stamp = self.get_clock().now().to_msg()
            self.state_msg.state = request.state
            self.publish()
        else:
            self.get_logger().warn(f"[{self.node_name}] {request.state} is not a valid state.")
        return response

    def publishState(self):
        self.pub_state.publish(self.state_msg)
        self.get_logger().info(f"[{self.node_name}] FSMState: {self.state_msg.state}")

    def publishBools(self):
        active_nodes = self._getActiveNodesOfState(self.state_msg.state)

        for node_name, srv_client in list(self.srv_dict.items()):
            msg = BoolStamped()
            msg.header.stamp = self.state_msg.header.stamp
            msg.data = bool(node_name in active_nodes)
            node_state = "ON" if msg.data else "OFF"
            
            if self.active_nodes is not None:
                if (node_name in active_nodes) == (node_name in self.active_nodes):
                    continue

            # Call service asynchronously
            if srv_client.service_is_ready():
                request = SetBool.Request()
                request.data = msg.data
                future = srv_client.call_async(request)
                # For simplicity, we don't wait for the response in this conversion

        self.active_nodes = copy.deepcopy(active_nodes)

    def updateLights(self):
        lights = self._getLightsofState(self.state_msg.state)
        if lights is not None:
            msg = String()
            msg.data = lights
            if self.changePattern.service_is_ready():
                request = ChangePattern.Request()
                request.pattern_name = msg
                future = self.changePattern.call_async(request)
            else:
                self.get_logger().warning(
                    f"[{self.node_name}] ChangePattern service is not ready, the led_emitter_node might not be running."
                    )

    def cbEvent(self, msg: BoolStamped, event_name: str):
        if msg.data == self.event_trigger_dict[event_name]:
            # Update timestamp
            self.state_msg.header.stamp = msg.header.stamp
            next_state = self._getNextState(self.state_msg.state, event_name)
            if next_state is not None:
                # Has a defined transition
                self.state_msg.state = next_state
                self.publish()

    def on_shutdown(self):
        self.get_logger().info(f"[{self.node_name}] Shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
