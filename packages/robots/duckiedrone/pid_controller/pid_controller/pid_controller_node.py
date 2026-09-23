# ruff: noqa: ARG002,COM812,D101,D102,D103,D107,E501,I001,PLR0915,PLR2004,SLF001,UP045,W292
"""ROS 2 PID controller node for Duckiedrone."""

from enum import IntEnum
from math import atan2, asin
import time
from typing import Optional

import rclpy
from duckietown_msgs.msg import DroneControl, PIDDiagnostics as PIDDebugInfo, PIDState
from geometry_msgs.msg import Pose, Quaternion, Twist
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import State as FCUState
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, Float32
from std_srvs.srv import SetBool

from .pid_math import PIDAxis, PIDController, quaternion_from_euler
from .three_dim_vec import Error, Position, RPY, Velocity


class DroneMode(IntEnum):
    DISARMED = 0
    ARMED = 1
    FLYING = 2


class PIDControllerNode(Node):
    """Control the drone using position/velocity PID loops."""

    def __init__(self) -> None:
        super().__init__("pid_controller_node")

        self._declare_parameters()

        self.frequency = float(self.get_parameter("frequency").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.hover_height = float(self.get_parameter("hover_height").value)

        self.previous_mode = DroneMode.DISARMED
        self.current_mode = DroneMode.DISARMED

        self.position_control = False
        self.last_position_control = False

        self.current_position = Position()
        self.desired_position = Position(z=self.hover_height)
        self.last_desired_position = Position(z=self.hover_height)

        self.current_velocity = Velocity()
        self.desired_velocity = Velocity()

        self.position_error = Error()
        self.velocity_error = Error()
        self.pid_error = Error()

        self.desired_velocity_travel_distance = 0.1
        self.desired_velocity_travel_time = 0.1
        self.desired_yaw_velocity_travel_time = 0.25
        self.desired_velocity_start_time: float | None = None
        self.desired_yaw_velocity_start_time: float | None = None

        self.pid = PIDController()
        self.pid.thrust.setpoint = self.hover_height
        self.lr_pid = PIDAxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0.0, control_range=(-10.0, 10.0))
        self.fb_pid = PIDAxis(kp=20.0, ki=5.0, kd=10.0, midpoint=0.0, control_range=(-10.0, 10.0))
        self._sync_pid_gains()

        self.last_pose_time: float | None = None
        self.desired_yaw_velocity = 0.0
        self.current_rpy = RPY()
        self.previous_rpy = RPY()
        self.current_state = Odometry()
        self.previous_state = Odometry()
        self.moving = False
        self.safety_threshold = 1.5
        self.lost = False
        self.absolute_desired_position = False
        self.path_planning = True
        self._last_fly_command = DroneControl()

        self.cmd_pub = self.create_publisher(AttitudeTarget, "~/commands", 1)
        self.debug_pub = self.create_publisher(PIDDebugInfo, "~/debug/pid", 10)
        self.position_control_pub = self.create_publisher(Bool, "~/position_control", 1)
        self.heartbeat_pub = self.create_publisher(Empty, "~/heartbeat", 1)

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._desired_height_pub = self.create_publisher(Float32, "~/desired/height", latched_qos)

        self.create_subscription(FCUState, "~/mode", self.current_mode_callback, 1)
        self.create_subscription(Odometry, "~/state", self.current_state_callback, 1)
        self.create_subscription(Pose, "desired/pose", self.desired_pose_callback, 1)
        self.create_subscription(Twist, "desired/twist", self.desired_twist_callback, 1)
        self.create_subscription(Bool, "camera_node/lost", self.lost_callback, 1)
        self.create_subscription(Empty, "reset_transform", self.reset_callback, 1)
        self.create_subscription(Bool, "~/position_control", self.position_control_callback, 1)

        self.set_mode_client = self.create_client(SetMode, "~/set_mode")
        self.takeoff_srv = self.create_service(SetBool, "~/takeoff", self.takeoff_srv_cb)
        self.land_srv = self.create_service(SetBool, "~/land", self.land_srv_cb)

        self.set_mode("GUIDED_NOGPS")
        self._desired_height_pub.publish(Float32(data=float(self.desired_position.z)))

        frequency = max(self.frequency, 1e-3)
        self._timer = self.create_timer(1.0 / frequency, self.control_loop_callback)

    def _declare_parameters(self) -> None:
        self.declare_parameter("frequency", 60.0)
        self.declare_parameter("max_height", 0.5)
        self.declare_parameter("hover_height", 0.3)
        self.declare_parameter("pitch_kp", 0.5)
        self.declare_parameter("pitch_ki", 0.0)
        self.declare_parameter("pitch_kd", 0.0)
        self.declare_parameter("roll_kp", 0.5)
        self.declare_parameter("roll_ki", 0.0)
        self.declare_parameter("roll_kd", 0.0)
        self.declare_parameter("yaw_kp", 0.0)
        self.declare_parameter("yaw_ki", 0.0)
        self.declare_parameter("yaw_kd", 0.0)
        self.declare_parameter("thrust_kp", 0.10)
        self.declare_parameter("thrust_ki", 0.05)
        self.declare_parameter("thrust_kd", 0.04)

    def _sync_pid_gains(self) -> None:
        self.pid.pitch.kp = float(self.get_parameter("pitch_kp").value)
        self.pid.pitch.ki = float(self.get_parameter("pitch_ki").value)
        self.pid.pitch.kd = float(self.get_parameter("pitch_kd").value)
        self.pid.roll.kp = float(self.get_parameter("roll_kp").value)
        self.pid.roll.ki = float(self.get_parameter("roll_ki").value)
        self.pid.roll.kd = float(self.get_parameter("roll_kd").value)
        self.pid.yaw.kp = float(self.get_parameter("yaw_kp").value)
        self.pid.yaw.ki = float(self.get_parameter("yaw_ki").value)
        self.pid.yaw.kd = float(self.get_parameter("yaw_kd").value)
        self.pid.thrust.kp = float(self.get_parameter("thrust_kp").value)
        self.pid.thrust.ki = float(self.get_parameter("thrust_ki").value)
        self.pid.thrust.kd = float(self.get_parameter("thrust_kd").value)

    def takeoff_srv_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            self.current_mode = DroneMode.FLYING
        else:
            self.current_mode = DroneMode.ARMED
        response.success = True
        response.message = f"Mode set to {self.current_mode.name}"
        return response

    def land_srv_cb(self, _request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.set_mode("LAND")
        response.success = True
        response.message = "Mode set to LAND"
        return response

    def set_mode(self, mode: str) -> None:
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("SetMode service is not available yet.")
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        start = time.monotonic()
        while rclpy.ok() and not future.done() and time.monotonic() - start < 5.0:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warning("Timed out calling the set mode service.")
            return

        response = future.result()
        if response is not None and response.mode_sent:
            self.get_logger().info("Successfully called the set mode service")
        else:
            self.get_logger().warning("Failed to call the set mode service")

    def current_state_callback(self, state: Odometry) -> None:
        self.previous_state = self.current_state
        self.current_state = state
        self.state_to_three_dim_vec_structs()

    def desired_pose_callback(self, msg: Pose) -> None:
        self.last_desired_position = Position(
            self.desired_position.x,
            self.desired_position.y,
            self.desired_position.z,
        )

        if self.absolute_desired_position:
            self.desired_position.x = msg.position.x
            self.desired_position.y = msg.position.y
            self.desired_position.z = (
                msg.position.z
                if 0.0 <= msg.position.z <= self.max_height * 0.8
                else self.last_desired_position.z
            )
        else:
            self.desired_position.x = self.current_position.x + msg.position.x
            self.desired_position.y = self.current_position.y + msg.position.y
            desired_z = self.last_desired_position.z + msg.position.z
            self.desired_position.z = (
                desired_z
                if 0.0 <= desired_z <= self.max_height * 0.8
                else self.last_desired_position.z
            )

        if self.desired_position != self.last_desired_position:
            self.moving = True
            self.get_logger().info("moving")

        self._desired_height_pub.publish(Float32(data=float(self.desired_position.z)))

    def desired_twist_callback(self, msg: Twist) -> None:
        self.desired_velocity.x = msg.linear.x
        self.desired_velocity.y = msg.linear.y
        self.desired_velocity.z = msg.linear.z
        self.desired_yaw_velocity = msg.angular.z
        self.desired_velocity_start_time = None
        self.desired_yaw_velocity_start_time = None
        self.get_logger().info(f"Desired_velocity set to {self.desired_velocity}")
        if self.path_planning:
            self.calculate_travel_time()

    def current_mode_callback(self, msg: FCUState) -> None:
        if msg.armed:
            if self.current_mode == DroneMode.DISARMED:
                self.current_mode = DroneMode.ARMED
        else:
            self.current_mode = DroneMode.DISARMED

    def position_control_callback(self, msg: Bool) -> None:
        self.position_control = msg.data
        if self.position_control:
            self.desired_position = Position(
                self.current_position.x,
                self.current_position.y,
                self.current_position.z,
            )
        if self.position_control != self.last_position_control:
            self.get_logger().info(f"Position control: {self.position_control}")
            self.last_position_control = self.position_control

    def reset_callback(self, _msg: Empty) -> None:
        self.current_position = Position(z=self.current_position.z)
        self.desired_position = Position(
            self.current_position.x,
            self.current_position.y,
            self.current_position.z,
        )
        self.desired_velocity = Velocity()

    def lost_callback(self, msg: Bool) -> None:
        self.lost = msg.data

    def control_loop_callback(self) -> None:
        if not self.safety_check():
            return

        self._sync_pid_gains()
        self.heartbeat_pub.publish(Empty())
        commands, thrust = self.step()

        if self.previous_mode == DroneMode.DISARMED:
            if self.current_mode == DroneMode.ARMED:
                self.reset()
                self.position_control_pub.publish(Bool(data=False))
                self.log_mode_transition()
                self.previous_mode = self.current_mode
            return

        if self.previous_mode == DroneMode.ARMED:
            if self.current_mode == DroneMode.FLYING:
                self.reset()
                self.log_mode_transition()
                self.previous_mode = self.current_mode
            elif self.current_mode == DroneMode.DISARMED:
                self.log_mode_transition()
                self.previous_mode = self.current_mode
            return

        if self.previous_mode == DroneMode.FLYING:
            if self.current_mode == DroneMode.FLYING:
                self.publish_control_cmd(commands, thrust)
            elif self.current_mode == DroneMode.DISARMED:
                self.log_mode_transition()
                self.previous_mode = self.current_mode
                self.pid.roll_low.init_i = self.pid.roll_low.integral
                self.pid.pitch_low.init_i = self.pid.pitch_low.integral

    def step(self) -> tuple[tuple[float, float, float], float]:
        current_time = time.monotonic()
        self.calc_error()
        if self.position_control:
            if self.position_error.xy_magnitude < self.safety_threshold and not self.lost:
                if self.moving:
                    if self.position_error.magnitude > 0.05:
                        self.pid_error = self.pid_error - self.velocity_error
                    else:
                        self.moving = False
                        self.get_logger().info("not moving")
            else:
                self.position_control_pub.publish(Bool(data=False))

        if self.desired_velocity.magnitude > 0.0 or abs(self.desired_yaw_velocity) > 0.0:
            self.adjust_desired_velocity()

        commands, thrust = self.pid.step(self.pid_error, current_time, self.desired_yaw_velocity)
        self._last_fly_command = DroneControl(
            roll=float(commands[0]),
            pitch=float(commands[1]),
            yaw=float(commands[2]),
            throttle=float(thrust),
        )
        self._publish_debug_info(thrust)
        return commands, thrust

    def state_to_three_dim_vec_structs(self) -> None:
        pose = self.current_state.pose.pose
        self.current_position.x = pose.position.x
        self.current_position.y = pose.position.y
        self.current_position.z = pose.position.z

        twist = self.current_state.twist.twist
        self.current_velocity.x = twist.linear.x
        self.current_velocity.y = twist.linear.y
        self.current_velocity.z = twist.linear.z

        self.previous_rpy = self.current_rpy
        self.current_rpy = RPY(*self._euler_from_quaternion(pose.orientation))

    def adjust_desired_velocity(self) -> None:
        current_time = time.monotonic()
        if self.desired_velocity_start_time is not None:
            if current_time - self.desired_velocity_start_time > self.desired_velocity_travel_time:
                self.desired_velocity.x = 0.0
                self.desired_velocity.y = 0.0
                self.desired_velocity_start_time = None
        else:
            self.desired_velocity_start_time = current_time

        if self.desired_yaw_velocity_start_time is not None:
            if current_time - self.desired_yaw_velocity_start_time > self.desired_yaw_velocity_travel_time:
                self.desired_yaw_velocity = 0.0
                self.desired_yaw_velocity_start_time = None
        else:
            self.desired_yaw_velocity_start_time = current_time

    def calc_error(self) -> None:
        pose_dt = 0.0
        current_time = time.monotonic()
        if self.last_pose_time is not None:
            pose_dt = current_time - self.last_pose_time
        self.last_pose_time = current_time

        self.velocity_error = Error(
            self.desired_velocity.x - self.current_velocity.x,
            self.desired_velocity.y - self.current_velocity.y,
            self.desired_velocity.z - self.current_velocity.z,
        )
        dz = self.desired_position.z - self.current_position.z

        self.pid_error = Error(self.velocity_error.x, self.velocity_error.y, dz)
        self.position_error = Error(
            self.desired_position.x - self.current_position.x,
            self.desired_position.y - self.current_position.y,
            self.desired_position.z - self.current_position.z,
        )

        if self.position_control:
            lr_step = self.lr_pid.step(self.position_error.x, pose_dt)
            fb_step = self.fb_pid.step(self.position_error.y, pose_dt)
            self.pid_error.x += lr_step
            self.pid_error.y += fb_step

    def calculate_travel_time(self) -> None:
        if self.desired_velocity.magnitude > 0.0:
            self.desired_velocity_travel_time = (
                self.desired_velocity_travel_distance / self.desired_velocity.xy_magnitude
            )
        else:
            self.desired_velocity_travel_time = 0.0

    def reset(self) -> None:
        self.position_error = Error()
        self.desired_position = Position(
            self.current_position.x,
            self.current_position.y,
            self.hover_height,
        )
        self.velocity_error = Error()
        self.desired_velocity = Velocity()
        self.pid.reset()
        self.lr_pid.reset()
        self.fb_pid.reset()

    def publish_control_cmd(self, commands: tuple[float, float, float], thrust: float) -> None:
        if thrust < 0.0 or thrust > 1.0:
            self.get_logger().error(f"Received thrust outside the range 0-1. Thrust: {thrust}")

        roll, pitch, yaw = commands
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        msg = AttitudeTarget()
        msg.thrust = float(thrust)
        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.cmd_pub.publish(msg)

    def _publish_debug_info(self, thrust: float) -> None:
        debug_info = PIDDebugInfo(
            current_position=self.current_position.as_ros_vector3(),
            desired_position=self.desired_position.as_ros_vector3(),
            position_error=self.position_error.as_ros_vector3(),
            current_velocity=self.current_velocity.as_ros_vector3(),
            desired_velocity=self.desired_velocity.as_ros_vector3(),
            velocity_error=self.velocity_error.as_ros_vector3(),
            pid_error=self.pid_error.as_ros_vector3(),
            fly_command=self._last_fly_command,
            pid_pitch=self.pid_axis_to_pid_state_msg(self.pid.pitch, self.pid_error.y),
            pid_roll=self.pid_axis_to_pid_state_msg(self.pid.roll, self.pid_error.x),
            pid_yaw=self.pid_axis_to_pid_state_msg(self.pid.yaw, self.desired_yaw_velocity),
            pid_throttle=self.pid_throttle_to_msg(self.pid_error.z),
            throttle_integral=float(self.pid.thrust.integral),
        )
        self.debug_pub.publish(debug_info)

    @staticmethod
    def pid_axis_to_pid_state_msg(pid: PIDAxis, error: float) -> PIDState:
        return PIDState(
            error=float(error),
            kp=float(pid.kp),
            kd=float(pid.kd),
            ki=float(pid.ki),
            proportional_output=float(pid._p),
            derivative_output=float(pid._d),
            integral_output=float(pid.integral),
            integral_state=float(pid.integral),
        )

    def pid_throttle_to_msg(self, error: float) -> PIDState:
        return PIDState(
            error=float(error),
            kp=float(self.pid.thrust.kp),
            kd=float(self.pid.thrust.kd),
            ki=float(self.pid.thrust.ki),
            proportional_output=float(self.pid.thrust.proportional_output),
            derivative_output=float(self.pid.thrust.derivative_output),
            integral_output=float(self.pid.thrust.integral_output),
            integral_state=float(self.pid.thrust.integral),
        )

    def log_mode_transition(self) -> None:
        self.get_logger().info(
            f"Transitioned from {self.previous_mode.name} to {self.current_mode.name}"
        )

    def safety_check(self) -> bool:
        if self.current_state.pose.pose.position.z > self.max_height:
            self.get_logger().info("disarming because drone is too high")
            self.previous_mode = DroneMode.DISARMED
            self.current_mode = DroneMode.DISARMED
            return False
        return True

    @staticmethod
    def _euler_from_quaternion(q: Quaternion) -> tuple[float, float, float]:
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PIDControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()