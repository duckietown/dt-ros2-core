"""Safety checks of the controller on a 1-D vertical drone, ROS stubbed."""

import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock

import pytest

for m in (
    "rclpy",
    "rclpy.qos",
    "rclpy.signals",
    "rclpy.time",
    "geometry_msgs",
    "geometry_msgs.msg",
    "mavros_msgs",
    "mavros_msgs.msg",
    "mavros_msgs.srv",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
):
    sys.modules[m] = MagicMock()


class Node:
    """Stand-in for rclpy.node.Node."""

    def __init__(self, name: str) -> None:
        """Do nothing."""

    def declare_parameter(self, name: str, value: object) -> NS:
        """Return the default."""
        return NS(value=value)

    def __getattr__(self, name: str) -> MagicMock:
        """Any other ROS call."""
        return MagicMock()


sys.modules["rclpy.node"] = types.SimpleNamespace(Node=Node)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pid_controller import pid_controller_node as H  # noqa: E402

HOVER, G, DT = 0.47, 9.81, 0.001
OK = NS(armed=True, mode="OFFBOARD", system_status=4)


class Sim:
    """1-D drone: thrust HOVER holds it, height seen 34 ms late."""

    def __init__(self, clock: list) -> None:
        """Create the node on the floor, disarmed."""
        self.clock = clock
        clock[0] = 1000.0
        self.n = H.PIDControllerNode()
        self.n.age = lambda m: getattr(m, "age", 0.0)
        self.z = self.v = self.t = self.nxt = 0.0
        self.hist = [0.0]
        self.pin = None
        self.lost = False
        self.n.state = NS(armed=False, mode="STABILIZED", system_status=3)

    def feed(self) -> None:
        """Height (unless lost) and flow heartbeat into the node."""
        if not self.lost:
            self.n.on_alt(NS(range=self.hist[max(0, len(self.hist) - 34)]))
        self.n.hb_t = self.clock[0]

    def ready(self) -> tuple[bool, str]:
        """Arm in OFFBOARD and take off."""
        self.step(2.0)
        self.n.act("arm")
        self.n.state = OK
        return self.n.act("takeoff")

    def step(self, secs: float) -> None:
        """Run the drone and the node for secs."""
        end = self.t + secs
        while self.t < end:
            if self.t >= self.nxt:
                self.nxt += 1 / H.RATE
                self.feed()
                self.n.loop()
            a = G * (self.n.thr / HOVER - 1) - 0.3 * self.v
            if self.z <= 0 and a < 0:
                a, self.v = 0.0, 0.0
            self.v += a * DT
            self.z = max(0.0, self.z + self.v * DT)
            if self.pin is not None and self.z >= self.pin:
                self.z, self.v = self.pin, min(self.v, 0.0)
            self.hist.append(self.z)
            self.t += DT
            self.clock[0] = 1000 + self.t


@pytest.fixture
def sim(monkeypatch: pytest.MonkeyPatch):  # noqa: ANN201
    """Factory of Sims on a fake clock."""
    clock = [1000.0]
    monkeypatch.setattr(H.time, "monotonic", lambda: clock[0])
    return lambda: Sim(clock)


def test_takeoff_guards(sim) -> None:  # noqa: ANN001
    """Takeoff only in OFFBOARD, fresh height, on the floor, with flow."""
    s = sim()
    s.step(2.0)
    s.n.state = NS(armed=True, mode="POSCTL", system_status=4)
    s.n.phase = "armed"
    assert not s.n.act("takeoff")[0]
    s.n.state = OK
    s.clock[0] += 1.0
    assert not s.n.act("takeoff")[0], "stale height"
    s.n.on_alt(NS(range=0.2))
    s.n.hb_t = s.clock[0]
    assert not s.n.act("takeoff")[0], "off the floor"
    s.n.on_alt(NS(range=0.01))
    s.n.hb_t = None
    assert not s.n.act("takeoff")[0], "no flow heartbeats"
    s.n.hb_t = s.clock[0]
    assert s.n.act("takeoff")[0]


def test_climb_holds_hover_height(sim) -> None:  # noqa: ANN001
    """A normal takeoff settles at hover_height."""
    s = sim()
    assert s.ready()[0]
    s.step(15.0)
    assert abs(s.z - s.n.target) < 0.03, f"z {s.z:.3f}"


@pytest.mark.parametrize(
    "state",
    [
        NS(armed=True, mode="CMODE(655360)", system_status=8),
        NS(armed=True, mode="AUTO.LAND", system_status=4),
    ],
)
def test_px4_leaves_offboard(sim, state) -> None:  # noqa: ANN001
    """PX4 termination or mode change: idle, thrust 0, no jump back."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.n.state = state
    s.step(0.2)
    assert s.n.phase == "idle"
    assert s.n.thr == 0.0
    s.n.state = OK
    s.step(1.0)
    assert s.n.thr == 0.0


def test_failsafe_left_on_disarm(sim) -> None:  # noqa: ANN001
    """The failsafe descent ends when PX4 disarms."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.n.phase, s.n.fs_since = "failsafe", s.clock[0]
    s.n.state = NS(armed=False, mode="OFFBOARD", system_status=4)
    s.step(0.2)
    assert s.n.phase == "idle"
    assert s.n.thr == 0.0


def test_land_cannot_delay_failsafe_kill(sim) -> None:  # noqa: ANN001
    """LAND while the height is lost keeps the 6 s kill timer."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.lost = True
    s.step(0.5)
    assert s.n.phase == "failsafe"
    fs_since = s.n.fs_since
    for _ in range(4):
        assert not s.n.act("land")[0]
        s.step(1.5)
    assert s.n.fs_since == fs_since
    assert s.n.phase == "idle"
    assert s.n.thr == 0.0


def test_land_after_height_returns_goes_down(sim) -> None:  # noqa: ANN001
    """LAND after the height returns goes down, not up first."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.lost = True
    s.step(0.7)
    s.lost = False
    s.step(0.2)
    assert s.n.phase == "failsafe"
    z0 = s.z
    assert s.n.act("land")[0]
    z_max = z0
    for _ in range(40):
        s.step(0.05)
        z_max = max(z_max, s.z)
    assert z_max < z0 + 0.05, f"climbed {z_max - z0:.3f} m"


def test_integral_bounded_when_pinned(sim) -> None:  # noqa: ANN001
    """Held at 15 cm for 60 s, the integral stays bounded; it then lands."""
    s = sim()
    s.pin = 0.15
    s.ready()
    s.step(60.0)
    assert abs(s.n.iz) <= (s.n.cap - s.n.K) / s.n.ki + 1e-9
    s.pin = None
    s.n.act("land")
    s.step(15.0)
    assert s.n.phase == "armed"
    assert s.z < 0.01


def test_nan_flow_dropped_and_d_reset(sim) -> None:  # noqa: ANN001
    """Non-finite flow is ignored; the D filter resets on stale flow."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.n.on_flow(NS(twist=NS(twist=NS(linear=NS(x=0.1, y=0.0)))))
    s.n.on_flow(NS(twist=NS(twist=NS(linear=NS(x=float("nan"), y=0.0)))))
    assert s.n.v == (0.1, 0.0)
    s.n.xy(s.clock[0])
    s.n.on_flow(NS(twist=NS(twist=NS(linear=NS(x=0.3, y=0.0)))))
    s.n.xy(s.clock[0] + 0.05)
    assert s.n.dx != 0.0
    s.n.xy(s.clock[0] + 1.0)
    assert s.n.dx == 0.0


def test_old_readings_ignored(sim) -> None:  # noqa: ANN001
    """Height and flow measured too long ago are ignored."""
    s = sim()
    s.step(2.0)
    s.n.on_alt(NS(range=0.2, age=H.H_STALE + 0.1))
    assert s.n.h != 0.2
    v = NS(linear=NS(x=0.5, y=0.0))
    s.n.on_flow(NS(twist=NS(twist=v), age=H.FLOW_STALE + 0.1))
    assert s.n.v != (0.5, 0.0)
    s.n.on_alt(NS(range=0.2, age=0.25))
    assert s.n.h == 0.2
    assert s.n.h_t == pytest.approx(s.clock[0] - 0.25)


def test_xy_on_needs_heartbeats(sim) -> None:  # noqa: ANN001
    """XY on is refused without flow heartbeats, XY off always works."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.n.hb_t = s.clock[0] - 1.0
    assert not s.n.act("xy_on")[0]
    assert s.n.act("xy_off")[0]


def test_exit_in_flight_lands(sim, monkeypatch) -> None:  # noqa: ANN001
    """Shutdown while flying lands before returning."""
    s = sim()
    s.ready()
    s.step(10.0)
    monkeypatch.setattr(H.rclpy, "ok", lambda: True)
    monkeypatch.setattr(H.rclpy, "spin_once", lambda *_, **__: s.step(0.05))
    H.land_before_exit(s.n)
    assert s.n.phase == "armed"
    assert s.z < 0.01


def test_exit_leaves_px4_takeover_alone(sim) -> None:  # noqa: ANN001
    """On exit, force-disarm only while PX4 is in OFFBOARD."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.n.state = NS(armed=True, mode="AUTO.LAND", system_status=4)
    s.step(0.2)
    s.n.cli_cmd = MagicMock()
    H.kill_on_exit(s.n)
    s.n.cli_cmd.call_async.assert_not_called()
    s.n.state = OK
    H.kill_on_exit(s.n)
    s.n.cli_cmd.call_async.assert_called_once()


def test_link_loss(sim) -> None:  # noqa: ANN001
    """No link ever: keeps flying; gap 1.5 s: flying; over 2 s: lands."""
    s = sim()
    s.ready()
    s.step(10.0)
    assert s.n.phase == "fly"
    s = sim()
    s.ready()
    for _ in range(200):
        s.n.link_t = s.clock[0]
        s.step(0.05)
    s.step(1.5)
    assert s.n.phase == "fly"
    s.step(1.0)
    assert s.n.phase == "land"
    s.step(15.0)
    assert s.n.phase == "armed"
    assert s.z < 0.01


def test_battery(sim) -> None:  # noqa: ANN001
    """A short dip flies on, 1 s under land_voltage lands, 0 is off."""
    s = sim()
    s.ready()
    s.step(10.0)
    s.n.vbat = 14.2
    s.step(0.5)
    assert s.n.phase == "fly"
    s.n.vbat = 14.5
    s.step(0.5)
    s.n.vbat = 14.2
    s.step(1.2)
    assert s.n.phase == "land"
    s = sim()
    s.n.land_v = 0.0
    s.ready()
    s.step(10.0)
    s.n.vbat = 13.0
    s.step(2.0)
    assert s.n.phase == "fly"
