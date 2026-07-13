"""Math helpers for the Duckiedrone PID controller."""

from dataclasses import dataclass
from math import sin, cos
from time import monotonic

from .three_dim_vec import Error


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a float to the given inclusive range."""
    return max(lower, min(value, upper))


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw to a quaternion tuple (x, y, z, w)."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = cos(half_roll)
    sr = sin(half_roll)
    cp = cos(half_pitch)
    sp = sin(half_pitch)
    cy = cos(half_yaw)
    sy = sin(half_yaw)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class PIDAxis:
    """Single PID axis with optional low-pass smoothing on the D term."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_range: tuple[float, float] = (1000.0, 2000.0),
        d_range: tuple[float, float] | None = None,
        control_range: tuple[float, float] = (1000.0, 2000.0),
        midpoint: float = 1500.0,
        smoothing: bool = True,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_range = i_range
        self.d_range = d_range
        self.control_range = control_range
        self.midpoint = midpoint
        self.smoothing = smoothing

        self.init_i = 0.0
        self._old_err: float | None = None
        self._p = 0.0
        self.integral = self.init_i
        self._d = 0.0
        self._dd = 0.0
        self._ddd = 0.0

    def reset(self) -> None:
        self._old_err = None
        self._p = 0.0
        self.integral = self.init_i
        self.init_i = 0.0
        self._d = 0.0
        self._dd = 0.0
        self._ddd = 0.0

    def step(self, err: float, delta_t: float) -> float:
        if delta_t <= 0.0:
            return self.midpoint

        if self._old_err is None:
            self._old_err = err

        self._p = err * self.kp
        self.integral += err * self.ki * delta_t
        if self.i_range is not None:
            self.integral = clamp(self.integral, self.i_range[0], self.i_range[1])

        self._d = (err - self._old_err) * self.kd / delta_t
        if self.d_range is not None:
            self._d = clamp(self._d, self.d_range[0], self.d_range[1])
        self._old_err = err

        if self.smoothing:
            self._d = (self._d * 8.0 + self._dd * 5.0 + self._ddd * 2.0) / 15.0
            self._ddd = self._dd
            self._dd = self._d

        raw_output = self._p + self.integral + self._d
        return clamp(raw_output + self.midpoint, self.control_range[0], self.control_range[1])


class SimplePIDController:
    """Minimal scalar PID with output limits."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        sample_time: float = 0.02,
        output_limits: tuple[float, float] = (0.0, 1.0),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.integral = 0.0
        self._last_error = 0.0
        self._last_time: float | None = None
        self.proportional_output = 0.0
        self.derivative_output = 0.0
        self.integral_output = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self.proportional_output = 0.0
        self.derivative_output = 0.0
        self.integral_output = 0.0

    def __call__(self, measurement: float) -> float:
        now = monotonic()
        if self._last_time is None:
            delta_t = self.sample_time
        else:
            delta_t = max(now - self._last_time, 1e-6)
        self._last_time = now

        error = self.setpoint - measurement
        self.proportional_output = self.kp * error
        self.integral += error * delta_t
        self.integral_output = self.ki * self.integral
        self.derivative_output = self.kd * ((error - self._last_error) / delta_t)
        self._last_error = error

        output = self.proportional_output + self.integral_output + self.derivative_output
        return clamp(output, self.output_limits[0], self.output_limits[1])


class PIDController:
    """Composite PID controller for roll, pitch, yaw, and thrust."""

    PID_SAMPLE_RATE = 50
    TARGET_ALTITUDE = 1.0

    def __init__(self):
        self.roll = PIDAxis(4.0, 1.0, 0.0, control_range=(1400.0, 1600.0), midpoint=1500.0, i_range=(-100.0, 100.0))
        self.roll_low = PIDAxis(0.0, 0.5, 0.0, control_range=(1400.0, 1600.0), midpoint=1500.0, i_range=(-150.0, 150.0))
        self.pitch = PIDAxis(4.0, 1.0, 0.0, control_range=(1400.0, 1600.0), midpoint=1500.0, i_range=(-100.0, 100.0))
        self.pitch_low = PIDAxis(0.0, 0.5, 0.0, control_range=(1400.0, 1600.0), midpoint=1500.0, i_range=(-150.0, 150.0))
        self.yaw = PIDAxis(0.0, 0.0, 0.0)
        self.thrust = SimplePIDController(0.10, 0.05, 0.04, setpoint=self.TARGET_ALTITUDE, sample_time=1.0 / self.PID_SAMPLE_RATE, output_limits=(0.0, 1.0))

        self.trim_controller_cap_plane = 0.05
        self.trim_controller_thresh_plane = 0.0001
        self._t: float | None = None

        self.roll_low.init_i = 0.31
        self.pitch_low.init_i = -1.05
        self.reset()

    def reset(self) -> None:
        self._t = None
        for pid in [self.roll, self.roll_low, self.pitch, self.pitch_low, self.yaw, self.thrust]:
            pid.reset()

    def step(self, error: Error, t: float, cmd_yaw_velocity: float = 0.0) -> tuple[tuple[float, float, float], float]:
        if self._t is None:
            time_elapsed = 1.0
        else:
            time_elapsed = t - self._t
        self._t = t

        cmd_roll = self.compute_axis_command(error.y, time_elapsed, pid_low=self.roll_low, pid=self.roll, trim_controller=self.trim_controller_cap_plane)
        cmd_pitch = self.compute_axis_command(error.x, time_elapsed, pid_low=self.pitch_low, pid=self.pitch, trim_controller=self.trim_controller_cap_plane)
        cmd_yaw = 1500.0 + cmd_yaw_velocity
        cmd_thrust = self.thrust(self.thrust.setpoint - error.z)
        return (cmd_roll, cmd_pitch, cmd_yaw), cmd_thrust

    def compute_axis_command(
        self,
        error: float,
        time_elapsed: float,
        pid: PIDAxis,
        pid_low: PIDAxis | None = None,
        trim_controller: float = 5.0,
    ) -> float:
        if pid_low is None:
            return pid.step(error, time_elapsed)

        if abs(error) < self.trim_controller_thresh_plane:
            pid_low.integral += pid.integral
            pid.integral = 0.0
            return pid_low.step(error, time_elapsed)

        if error > trim_controller:
            pid_low.step(trim_controller, time_elapsed)
        elif error < -trim_controller:
            pid_low.step(-trim_controller, time_elapsed)
        else:
            pid_low.step(error, time_elapsed)
        return pid.step(error, time_elapsed)