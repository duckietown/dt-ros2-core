"""ROS 2 height and velocity controller for the Duckiedrone on PX4.

Streams an attitude, a thrust and a neutral manual control at RATE in
OFFBOARD. It never arms by itself: arm, takeoff, land, disarm, kill and
the XY mode are Trigger services (~/arm, ~/takeoff, ...).

Height: u = thrust_hover + kp e + ki I + kd de/dt on the altitude node
height (0 = floor at its startup). Takeoff walks the setpoint up at
CLIMB_RATE, thrust changes at most SLEW per cycle and is capped at
thrust_max, and the integral only learns once airborne, bounded so it
cannot ask for more than thrust_max.

Velocity: drives the optical flow velocity to 0 with
a = kp e + ki I + kd de/dt [m/s^2], pitch = atan(a_x / g),
roll = -atan(a_y / g), each capped at TILT_CAP. It runs only when
airborne above XY_H_MIN and ramps in over XY_RAMP. A flow message older
than FLOW_STALE commands level. With xy_freeze_error the integrators
stop learning while the speed error is larger than that, and for
FREEZE_HOLD after, so a push is braked by kp alone.

Mode "shadow" computes the tilt but sends level.

Failsafes:
- PX4 leaves OFFBOARD, terminates or disarms: thrust 0, phase idle.
- Height older than H_STALE: level, descend at thrust_hover - 0.03,
  kill after FAILSAFE_MAX.
- Height above max_height, tilt at the cap for TILT_CAP_S, battery
  under land_voltage for BAT_LOW_S, or ~/link silent for LINK_LOST
  once it has been seen: land.
- Takeoff only armed in OFFBOARD, on the floor, with a fresh height
  (and flow heartbeats when xy_at_takeoff). XY on needs flow heartbeats.
- Shutdown in flight lands first.

Known defect: landing drops the lean and the learned trim at once, so
the drone slides back while it lands.

Units: metres, seconds, radians, thrust 0 to 1.

ROS parameters:
    hover_height [m]: height to hold.
    max_height [m]: lands above this.
    thrust_hover: thrust that about holds the drone up (feed-forward).
    thrust_max: thrust cap.
    thrust_kp [1/m], thrust_ki [1/(m s)], thrust_kd [s/m]: height PID.
    xy_kp [1/s], xy_ki [1/s^2], xy_kd: velocity PID, output in m/s^2.
    xy_freeze_error [m/s]: speed error above which the velocity
        integrators stop learning; 0 = off.
    xy_at_takeoff: switch velocity control on at takeoff.
    land_voltage [V]: lands when the battery stays under it in flight;
        0 = off.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import ManualControl, State, Thrust
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

G = 9.81  # [m/s^2]
RATE = 20.0  # control loop [Hz]
CLIMB_RATE = 0.10  # setpoint speed during takeoff and landing [m/s]
SLEW = 0.02  # largest thrust change per cycle
AIRBORNE_DH = 0.03  # height over the floor where the integral starts [m]
LAND_CUT_H = 0.10  # height over the floor where landing cuts thrust [m]
LAND_CUT_S = 1.0  # time to ramp thrust to 0 when landing [s]
H_STALE = 0.3  # [s]
FAILSAFE_MAX = 6.0  # [s]
FLOW_STALE = 0.3  # [s]
HB_STALE = 0.5  # flow heartbeat [s]
TAKEOFF_H_MAX = 0.05  # takeoff only below this height [m]
PX4_TERMINATION = 8  # MAV_STATE_FLIGHT_TERMINATION
LINK_LOST = 2.0  # [s]
BAT_LOW_S = 1.0  # [s]
XY_H_MIN = 0.10  # [m]
XY_RAMP = 0.3  # [s]
TILT_CAP = math.radians(5.0)
TILT_CAP_S = 2.0  # [s]
FREEZE_HOLD = 2.0  # [s]
D_TAU = 0.1  # low-pass time constant of the velocity D term [s]
SP_EPS = 1e-6  # setpoint reached [m]
KILL = (400, 21196.0)  # MAV_CMD_COMPONENT_ARM_DISARM, force


def quat_rpy(roll: float, pitch: float, yaw: float) -> tuple:
    """Quaternion (x, y, z, w) of roll, pitch, yaw [rad]."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy(q: tuple) -> tuple:
    """Roll, pitch, yaw [rad] of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def clip(v: float, lim: float) -> float:
    """Clip v to [-lim, lim]."""
    return max(-lim, min(lim, v))


class PIDControllerNode(Node):
    """Height and velocity controller."""

    def __init__(self) -> None:  # noqa: PLR0915
        """Read the parameters and create the ROS interfaces."""
        super().__init__("pid_controller_node")
        param = self.declare_parameter
        self.target = param("hover_height", 0.6).value
        self.h_max = param("max_height", 0.8).value
        self.K = param("thrust_hover", 0.44).value
        self.cap = param("thrust_max", 0.52).value
        self.kp = param("thrust_kp", 0.35).value
        self.ki = param("thrust_ki", 0.15).value
        self.kd = param("thrust_kd", 0.10).value
        self.kp_xy = param("xy_kp", 2.0).value
        self.ki_xy = param("xy_ki", 0.4).value
        self.kd_xy = param("xy_kd", 0.15).value
        self.freeze_err = param("xy_freeze_error", 0.20).value
        self.xy_auto = param("xy_at_takeoff", True).value
        self.land_v = param("land_voltage", 14.3).value

        self.phase = "idle"
        self.state = State()
        self.q = (0.0, 0.0, 0.0, 1.0)
        self.yaw_hold = None
        self.h = self.h_t = None
        self.h_ground = None
        self.sp = 0.0
        self.thr = 0.0
        self.iz = 0.0
        self.e_prev = None
        self.cut_rate = None
        self.fs_since = None
        self.xy_mode = "off"
        self.alpha = 0.0
        self.v = None
        self.v_t = 0.0
        self.hb_t = None
        self.link_t = None
        self.vbat = None
        self.bat_low_since = None
        self.ix = self.iy = 0.0
        self.calm_since = None
        self.frozen = False
        self.e_last = None
        self.dx = self.dy = 0.0
        self.ax = self.ay = 0.0
        self.cap_since = None
        self.t0 = time.monotonic()

        sensor = qos_profile_sensor_data
        self.pub_att = self.create_publisher(PoseStamped, "~/attitude", 10)
        self.pub_thr = self.create_publisher(Thrust, "~/thrust", 10)
        self.pub_mc = self.create_publisher(
            ManualControl,
            "~/manual_control",
            10,
        )
        self.create_subscription(State, "~/fcu_state", self.on_state, 10)
        self.create_subscription(Imu, "~/imu", self.on_imu, sensor)
        self.create_subscription(Range, "~/altitude", self.on_alt, sensor)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "~/velocity",
            self.on_flow,
            sensor,
        )
        self.create_subscription(Empty, "~/flow_heartbeat", self.on_hb, 1)
        self.create_subscription(Empty, "~/link", self.on_link, 1)
        self.create_subscription(
            BatteryState,
            "~/battery",
            self.on_battery,
            sensor,
        )
        self.cli_mode = self.create_client(SetMode, "~/set_mode")
        self.cli_arm = self.create_client(CommandBool, "~/arming")
        self.cli_cmd = self.create_client(CommandLong, "~/command")
        for name in (
            "arm",
            "takeoff",
            "land",
            "kill",
            "disarm",
            "xy_on",
            "xy_off",
            "xy_shadow",
        ):
            self.create_service(Trigger, f"~/{name}", self.service(name))
        self.create_timer(1.0 / RATE, self.loop)
        self.get_logger().info(
            f"hover {self.target} m, thrust {self.K} (max {self.cap}), "
            f"height PID {self.kp} {self.ki} {self.kd}, "
            f"velocity PID {self.kp_xy} {self.ki_xy} {self.kd_xy}, "
            f"freeze {self.freeze_err} m/s, land at {self.land_v} V",
        )

    # ---- inputs

    def on_state(self, m: State) -> None:
        """Keep the PX4 state."""
        self.state = m

    def on_imu(self, m: Imu) -> None:
        """Keep the attitude."""
        o = m.orientation
        self.q = (o.x, o.y, o.z, o.w)

    def age(self, m: Range | TwistWithCovarianceStamped) -> float:
        """Seconds since the message was measured, from its stamp."""
        dt = self.get_clock().now() - Time.from_msg(m.header.stamp)
        return max(0.0, dt.nanoseconds * 1e-9)

    def on_alt(self, m: Range) -> None:
        """Keep the height, unless it is already older than H_STALE."""
        age = self.age(m)
        if math.isfinite(m.range) and age < H_STALE:
            self.h, self.h_t = m.range, time.monotonic() - age

    def on_flow(self, m: TwistWithCovarianceStamped) -> None:
        """Keep the velocity [m/s], body forward and left, if fresh."""
        v = (m.twist.twist.linear.x, m.twist.twist.linear.y)
        age = self.age(m)
        if math.isfinite(v[0]) and math.isfinite(v[1]) and age < FLOW_STALE:
            self.v, self.v_t = v, time.monotonic() - age

    def on_hb(self, _: Empty) -> None:
        """Note a flow heartbeat."""
        self.hb_t = time.monotonic()

    def on_link(self, _: Empty) -> None:
        """Note a link message."""
        self.link_t = time.monotonic()

    def on_battery(self, m: BatteryState) -> None:
        """Keep the battery voltage [V]."""
        self.vbat = m.voltage

    def flow_alive(self) -> bool:
        """Whether the flow node sends heartbeats."""
        return (
            self.hb_t is not None and time.monotonic() - self.hb_t < HB_STALE
        )

    # ---- actions

    def service(self, name: str):  # noqa: ANN201
        """Trigger callback that runs one action."""

        def cb(_: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
            res.success, res.message = self.act(name)
            return res

        return cb

    def act(self, a: str) -> tuple[bool, str]:  # noqa: C901, PLR0911, PLR0912
        """Run one action; return whether it was accepted and why."""
        if a != "takeoff" or self.state.armed:
            self.get_logger().info(
                f"action {a} (phase {self.phase}, h {self.h})",
            )
        if a == "arm":
            if self.state.armed:
                return False, "already armed"
            self.yaw_hold = rpy(self.q)[2]
            self.cli_mode.call_async(SetMode.Request(custom_mode="OFFBOARD"))
            self.cli_arm.call_async(CommandBool.Request(value=True))
            self.phase = "armed"
            return True, (
                "OFFBOARD + arm requested, "
                f"yaw hold {math.degrees(self.yaw_hold):.1f} deg"
            )
        if a == "takeoff":
            if not self.state.armed or self.state.mode != "OFFBOARD":
                return False, f"not armed in OFFBOARD (mode {self.state.mode})"
            if self.phase != "armed":
                return (
                    False,
                    f"already {self.phase}: takeoff only from the ground",
                )
            if self.h_t is None or time.monotonic() - self.h_t > H_STALE:
                return False, "no fresh height"
            if self.h > TAKEOFF_H_MAX:
                return (
                    False,
                    f"height {self.h:.2f} m: takeoff only from the floor",
                )
            if self.xy_auto and not self.flow_alive():
                return (
                    False,
                    "no flow heartbeats: velocity control would be blind",
                )
            self.h_ground, self.sp, self.iz, self.e_prev = (
                self.h,
                self.h,
                0.0,
                None,
            )
            self.phase = "fly"
            if self.xy_auto:
                self.act("xy_on")
            xy = ", xy on" if self.xy_auto else ""
            return True, f"climbing to {self.target} m{xy}"
        if a == "land":
            if self.phase in ("fly", "failsafe"):
                self.phase, self.cut_rate = "land", None
            return True, "landing"
        if a == "disarm":
            if self.phase in ("fly", "land", "failsafe"):
                return (
                    False,
                    f"{self.phase}: use LAND, or KILL to stop the motors",
                )
            self.cli_arm.call_async(CommandBool.Request(value=False))
            self.phase, self.thr = "idle", 0.0
            return True, "disarm requested"
        if a == "kill":
            command, magic = KILL
            self.cli_cmd.call_async(
                CommandLong.Request(command=command, param1=0.0, param2=magic),
            )
            self.phase, self.thr, self.xy_mode = "idle", 0.0, "off"
            return True, "force-disarm sent"
        if a.startswith("xy_"):
            if a != "xy_off" and not self.flow_alive():
                return False, "no flow heartbeats: is the camera running?"
            self.xy_mode = {
                "xy_on": "on",
                "xy_off": "off",
                "xy_shadow": "shadow",
            }[a]
            self.alpha, self.ix, self.iy = 0.0, 0.0, 0.0
            self.calm_since, self.e_last, self.dx, self.dy = (
                None,
                None,
                0.0,
                0.0,
            )
            return True, f"xy {self.xy_mode}"
        return False, f"unknown action {a}"

    # ---- RATE loop

    def loop(self) -> None:
        """Check the failsafes, then compute and send one command."""
        now = time.monotonic()
        h_ok = self.h_t is not None and now - self.h_t < H_STALE
        fc_away = (
            self.state.mode != "OFFBOARD"
            or self.state.system_status == PX4_TERMINATION
        )
        flying = self.phase in ("fly", "land", "failsafe")
        if flying and (not self.state.armed or fc_away) and now - self.t0 > 1:
            if self.state.armed:
                self.get_logger().warn(
                    f"PX4 left OFFBOARD (mode {self.state.mode}, status "
                    f"{self.state.system_status}): thrust 0, KILL or DISARM",
                )
            self.phase, self.xy_mode = "idle", "off"
        if self.phase in ("fly", "land") and not h_ok:
            self.get_logger().warn("height stale: failsafe descent")
            self.phase, self.fs_since, self.xy_mode = "failsafe", now, "off"
        link_lost = self.link_t is not None and now - self.link_t > LINK_LOST
        if self.phase == "fly" and link_lost:
            self.get_logger().warn(f"no link for {LINK_LOST} s: landing")
            self.act("land")
        low = (
            self.phase == "fly"
            and self.land_v
            and self.vbat is not None
            and 0 < self.vbat < self.land_v
        )
        self.bat_low_since = (self.bat_low_since or now) if low else None
        if low and now - self.bat_low_since > BAT_LOW_S:
            self.get_logger().warn(
                f"battery {self.vbat:.2f} V under {self.land_v} V: landing",
            )
            self.act("land")
        if self.phase == "fly" and self.h is not None and self.h > self.h_max:
            self.get_logger().warn(f"height {self.h:.2f} above max: landing")
            self.act("land")

        u = self.altitude(now)
        self.thr = max(0.0, min(self.cap, self.thr + clip(u - self.thr, SLEW)))
        if self.phase in ("idle", "armed"):
            self.thr = 0.0

        roll, pitch = self.xy(now)
        if self.xy_mode != "on":
            roll, pitch = 0.0, 0.0
        self.publish(roll, pitch)

    def altitude(self, now: float) -> float:
        """Thrust the height loop asks for."""
        if self.phase == "failsafe":
            if self.fs_since and now - self.fs_since > FAILSAFE_MAX:
                self.act("kill")
            return self.K - 0.03
        if self.phase not in ("fly", "land"):
            return 0.0
        step = CLIMB_RATE / RATE
        if self.phase == "fly":
            if self.sp < self.target:
                self.sp = min(self.target, self.sp + step)
            else:
                self.sp = max(self.target, self.sp - step)
        else:
            self.sp = max(self.h_ground, self.sp - step)
            if self.cut_rate is None and self.h < self.h_ground + LAND_CUT_H:
                self.cut_rate = max(self.thr, 0.01) / (LAND_CUT_S * RATE)
                self.get_logger().info(
                    f"h {self.h:.2f} m: thrust {self.thr:.2f} to 0 "
                    f"over {LAND_CUT_S} s",
                )
            if self.cut_rate is not None:
                if self.thr - self.cut_rate <= 0.0:
                    self.phase, self.xy_mode, self.cut_rate = (
                        "armed",
                        "off",
                        None,
                    )
                    self.get_logger().info(
                        "landed: thrust 0, disarm when ready",
                    )
                    return 0.0
                return self.thr - self.cut_rate
        e = self.sp - self.h
        de = 0.0 if self.e_prev is None else (e - self.e_prev) * RATE
        self.e_prev = e
        if self.h > self.h_ground + AIRBORNE_DH and self.ki:
            self.iz = clip(self.iz + e / RATE, (self.cap - self.K) / self.ki)
        return self.K + self.kp * e + self.ki * self.iz + self.kd * de

    def xy(self, now: float) -> tuple[float, float]:
        """Roll and pitch [rad] the velocity loop asks for."""
        ready = (
            self.phase == "fly"
            and self.xy_mode != "off"
            and self.h is not None
            and self.h > XY_H_MIN
            and (self.xy_auto or abs(self.sp - self.target) < SP_EPS)
        )
        fresh = self.v is not None and now - self.v_t < FLOW_STALE
        if not ready or not fresh:
            self.alpha = 0.0
            self.cap_since = None
            self.e_last = None
            self.dx = self.dy = 0.0
            return 0.0, 0.0
        dt = 1.0 / RATE
        self.alpha = min(1.0, self.alpha + dt / XY_RAMP)
        ex, ey = -self.v[0], -self.v[1]
        if self.e_last is not None:
            k = dt / (D_TAU + dt)
            self.dx += k * ((ex - self.e_last[0]) / dt - self.dx)
            self.dy += k * ((ey - self.e_last[1]) / dt - self.dy)
        self.e_last = (ex, ey)
        a_lim = G * math.tan(TILT_CAP)
        fe = self.freeze_err
        if fe and math.hypot(ex, ey) > fe:
            self.calm_since = None
        elif self.calm_since is None:
            self.calm_since = now
        self.frozen = bool(fe) and (
            self.calm_since is None or now - self.calm_since < FREEZE_HOLD
        )
        if self.alpha >= 1.0 and not self.frozen and self.ki_xy:
            self.ix = clip(self.ix + ex * dt, a_lim / self.ki_xy)
            self.iy = clip(self.iy + ey * dt, a_lim / self.ki_xy)
        self.ax = self.kp_xy * ex + self.ki_xy * self.ix + self.kd_xy * self.dx
        self.ay = self.kp_xy * ey + self.ki_xy * self.iy + self.kd_xy * self.dy
        pitch = clip(self.alpha * math.atan(self.ax / G), TILT_CAP)
        roll = clip(-self.alpha * math.atan(self.ay / G), TILT_CAP)
        if max(abs(pitch), abs(roll)) >= TILT_CAP - 1e-6:
            self.cap_since = self.cap_since or now
            if self.xy_mode == "on" and now - self.cap_since > TILT_CAP_S:
                self.get_logger().warn("tilt at cap for 2 s: XY off, landing")
                self.xy_mode = "off"
                self.act("land")
        else:
            self.cap_since = None
        return roll, pitch

    # ---- output

    def publish(self, roll: float, pitch: float) -> None:
        """Send the attitude, thrust and a neutral manual control."""
        stamp = self.get_clock().now().to_msg()
        a = PoseStamped()
        a.header.stamp, a.header.frame_id = stamp, "map"
        yaw = self.yaw_hold if self.yaw_hold is not None else rpy(self.q)[2]
        o = a.pose.orientation
        o.x, o.y, o.z, o.w = quat_rpy(roll, pitch, yaw)
        self.pub_att.publish(a)
        t = Thrust()
        t.header.stamp, t.thrust = stamp, float(self.thr)
        self.pub_thr.publish(t)
        mc = ManualControl()
        mc.header.stamp = stamp
        self.pub_mc.publish(mc)


def land_before_exit(node: PIDControllerNode) -> None:
    """Land before exiting when in flight (up to 12 s)."""
    if node.phase not in ("fly", "land", "failsafe"):
        return
    node.get_logger().warn("exit while flying: landing first")
    node.act("land")
    t_end = time.monotonic() + 12.0
    while (
        rclpy.ok()
        and node.phase in ("land", "failsafe")
        and time.monotonic() < t_end
    ):
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args: list[str] | None = None) -> None:
    """Run the controller; on exit land first, then force-disarm."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PIDControllerNode()
    try:
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        land_before_exit(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.act("kill")
        t_end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
