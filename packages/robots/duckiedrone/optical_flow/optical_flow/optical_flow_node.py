"""ROS 2 optical flow node for Duckiedrone.

Publishes the drone's velocity over the floor [m/s] from the bottom
camera, the IMU attitude and the altitude node's height.

Per frame:
1. Decode the JPEG to a half-size gray image.
2. Find corners in the floor band of the previous frame and track them
   into the new one with Lucas-Kanade. Keep a track when tracking it
   back lands within FB_MAX_ERROR of where it started.
3. Undistort both ends of each kept track (fisheye), turn them into
   rays through the camera mount and the IMU attitude of their own
   frame, and extend the rays to the floor at that frame's camera
   height. The median floor motion, sign flipped, is the drone's
   displacement, turned into the current heading.
4. Remove the camera's own sideways motion from turning (lever arm).
5. Reject the frame if the camera is too low, too few tracks survive,
   the tracks disagree, or the speed is impossible.
6. Publish the velocity, stamped at the exposure time. Rejected frames
   publish nothing.

Frames: body x forward, y left, z up. World = the IMU's gravity-aligned
frame (ENU from mavros). Camera x right, y down, z out of the lens.
Units: metres, seconds, radians unless named _deg or [px].

ROS parameters:
    floor_rows [px, full size]: first and last image row of the floor
        band where corners are searched.
    max_corners: corners searched per frame.
    camera_delay [s]: image stamp minus exposure time.

Camera calibration, read at startup from the robot's files in
/data/config/calibrations (the node does not start without them):
    camera_intrinsic/<robot>.yaml: camera matrix and fisheye
        (equidistant) distortion of the full image.
    camera_extrinsic/<robot>.yaml: homography to the floor, taken with
        the drone landed. It gives the camera tilt down, turn to the
        right, distance in front of the body origin, and height above
        the floor where the altitude node reads 0.

The velocity is published in <robot>/base_stabilized: base_link with
roll and pitch removed.
"""

import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TwistWithCovarianceStamped
from numpy.typing import NDArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, Range
from std_msgs.msg import Empty

DECODE_SCALE = 2  # frames are decoded at half size
LK_WIN_SIZE = (21, 21)  # [px, half size]
LK_MAX_LEVEL = 3
LK_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01)
CORNER_QUALITY = 0.01
CORNER_MIN_DISTANCE = 8  # [px, half size]
MASK_MARGIN = 8  # [px, half size] left and right edge left out
FB_MAX_ERROR = 0.5  # [px, half size] forward-backward check
MIN_VECTORS = 5  # fewer kept tracks: no displacement for this frame
RAY_MIN_DOWN = 1e-3  # rays flatter than this never reach the floor

GATE_H_MIN = 0.15  # [m] camera height below this: "low"
GATE_N_MIN = 20  # fewer tracks: "corners"
GATE_SPREAD_STEP = 0.5  # spread above this share of the step ...
GATE_SPREAD_HEIGHT = 0.02  # ... plus this share of the height: "spread"
GATE_V_MAX = 1.5  # [m/s] faster: "speed"
SIGMA_MIN = 0.03  # [m/s] smallest velocity standard deviation
SIGMA_SPREAD_FACTOR = 1.25  # median absolute spread to sigma

IMU_BUFFER = 500  # samples
HEIGHT_BUFFER = 100  # samples
HEIGHT_STALE = 0.3  # [s] newest height older than this: no height
IMU_STALE = 0.3  # [s] newest attitude older than this: no attitude
SLERP_LINEAR_DOT = 0.9995
MIN_DT = 1e-6  # [s]

CALIBRATIONS = Path("/data/config/calibrations")


def stamp_to_sec(msg: Imu | Range | CompressedImage) -> float:
    """Header stamp of a message [s]."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def quat_to_matrix(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (xx + zz), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (xx + yy)],
        ],
    )


def slerp(
    q0: NDArray[np.float64],
    q1: NDArray[np.float64],
    a: float,
) -> NDArray[np.float64]:
    """Interpolate between two quaternions, a = 0 gives q0."""
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > SLERP_LINEAR_DOT:
        q = q0 + a * (q1 - q0)
    else:
        th = math.acos(dot)
        q = (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(
            th,
        )
    return q / np.linalg.norm(q)


def camera_to_body(
    pitch_deg: float,
    yaw_right_deg: float,
) -> NDArray[np.float64]:
    """Rotation from camera axes (x right, y down, z out) to body.

    Columns are the camera axes in body coordinates. Level camera
    looking forward: right = -y, down = -z, out = +x. Tilting it down
    by pitch turns out towards -z and down towards -x.
    """
    th = math.radians(pitch_deg)
    level = np.array(
        [
            [0.0, -math.sin(th), math.cos(th)],
            [-1.0, 0.0, 0.0],
            [0.0, -math.cos(th), -math.sin(th)],
        ],
    )
    yaw = -math.radians(yaw_right_deg)
    rot_z = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
    )
    return rot_z @ level


def camera_pose(h: list[float]) -> tuple[float, float, float, float]:
    """Camera pitch down [deg], yaw right [deg], ahead [m], height [m].

    h is the extrinsic homography from undistorted normalised image
    coordinates to the floor (x forward, y left), taken with the drone
    landed. Its inverse holds the floor axes and origin in camera axes,
    up to a sign; the right sign puts the camera above the floor.
    """
    m = np.linalg.inv(np.array(h, dtype=np.float64).reshape(3, 3))
    m /= np.linalg.norm(m[:, 0])
    for sign in (1.0, -1.0):
        r1, r2, t = sign * m[:, 0], sign * m[:, 1], sign * m[:, 2]
        rot = np.column_stack([r1, r2, np.cross(r1, r2)])
        centre = -rot.T @ t
        if centre[2] > 0:
            break
    else:
        msg = "the homography does not put the camera above the floor"
        raise ValueError(msg)
    axis = rot.T[:, 2] / np.linalg.norm(rot.T[:, 2])
    pitch = math.degrees(math.asin(-axis[2]))
    yaw_right = -math.degrees(math.atan2(axis[1], axis[0]))
    return pitch, yaw_right, float(centre[0]), float(centre[2])


def load_calibration(
    robot: str,
) -> tuple[NDArray[np.float64], NDArray[np.float64], tuple[float, ...]]:
    """Lens matrix, fisheye distortion and camera pose of a robot.

    Read from the robot's intrinsic and extrinsic calibration files.
    Raises RuntimeError naming the file when one is missing or wrong.
    """
    files = {
        kind: CALIBRATIONS / f"camera_{kind}" / f"{robot}.yaml"
        for kind in ("intrinsic", "extrinsic")
    }
    try:
        with files["intrinsic"].open() as f:
            lens = yaml.safe_load(f)
        if lens["distortion_model"] != "equidistant":
            model = lens["distortion_model"]
            msg = f"distortion model {model}, not equidistant"
            raise ValueError(msg)
        k = np.array(lens["camera_matrix"]["data"], dtype=np.float64)
        d = np.array(lens["distortion_coefficients"]["data"], dtype=np.float64)
        k, d = k.reshape(3, 3), d.reshape(4)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as e:
        msg = f"bad intrinsic calibration {files['intrinsic']}: {e!r}"
        raise RuntimeError(msg) from e
    try:
        with files["extrinsic"].open() as f:
            pose = camera_pose(yaml.safe_load(f)["homography"])
    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        np.linalg.LinAlgError,
        yaml.YAMLError,
    ) as e:
        msg = f"bad extrinsic calibration {files['extrinsic']}: {e!r}"
        raise RuntimeError(msg) from e
    return k, d, pose


class OpticalFlowNode(Node):
    """Measure the speed of the drone over the floor from the camera."""

    def __init__(self) -> None:
        """Initialize parameters, state, subscribers and publishers."""
        super().__init__("optical_flow_node")

        self._rows = self.declare_parameter("floor_rows", [430, 630]).value
        self._max_corners = self.declare_parameter("max_corners", 150).value
        self._delay = self.declare_parameter("camera_delay", 0.0).value

        try:
            self._k, self._d, pose = load_calibration(
                self.get_namespace().strip("/"),
            )
        except RuntimeError as e:
            self.get_logger().fatal(f"Not starting: {e}")
            raise
        pitch, yaw, self._ahead, self._height_landed = pose
        self._camera_to_body = camera_to_body(pitch, yaw)
        self.get_logger().info(
            f"Camera: f {self._k[0, 0]:.1f} px, pitch down {pitch:.1f} deg, "
            f"yaw right {yaw:.1f} deg, ahead {self._ahead:.3f} m, "
            f"landed height {self._height_landed:.3f} m",
        )
        # base_link with roll and pitch removed: the velocity is along the
        # heading, not along the tilted body axes
        self._frame_id = (
            self.get_namespace().strip("/") + "/base_stabilized"
        )

        self._imu: deque[tuple[float, NDArray[np.float64], float]] = deque(
            maxlen=IMU_BUFFER,
        )
        self._heights: deque[tuple[float, float]] = deque(
            maxlen=HEIGHT_BUFFER,
        )
        self._prev: tuple[float, NDArray, NDArray, float] | None = None
        self._mask: NDArray | None = None

        self.create_subscription(
            CompressedImage,
            "~/image",
            self.image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "~/imu",
            self.imu_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            "~/altitude",
            self.altitude_cb,
            qos_profile_sensor_data,
        )

        self._pub = self.create_publisher(
            TwistWithCovarianceStamped,
            "~/velocity",
            1,
        )
        self._heartbeat = self.create_publisher(Empty, "~/heartbeat", 1)

    def imu_cb(self, msg: Imu) -> None:
        """Store the stamped attitude and yaw rate."""
        q = msg.orientation
        self._imu.append(
            (
                stamp_to_sec(msg),
                np.array([q.x, q.y, q.z, q.w]),
                msg.angular_velocity.z,
            ),
        )

    def altitude_cb(self, msg: Range) -> None:
        """Store the stamped height above the floor."""
        self._heights.append((stamp_to_sec(msg), msg.range))

    def attitude_at(self, t: float) -> NDArray[np.float64]:
        """Body-to-world rotation at time t, interpolated (slerp)."""
        ts = np.array([s[0] for s in self._imu])
        if len(ts) < 2:  # noqa: PLR2004
            return quat_to_matrix(self._imu[-1][1])
        i = int(np.clip(np.searchsorted(ts, t), 1, len(ts) - 1))
        a = float(
            np.clip((t - ts[i - 1]) / max(ts[i] - ts[i - 1], MIN_DT), 0, 1),
        )
        return quat_to_matrix(slerp(self._imu[i - 1][1], self._imu[i][1], a))

    def mean_yaw_rate(self, t0: float, t1: float) -> float:
        """Mean yaw rate [rad/s] of the IMU samples from t0 to t1."""
        rates = [wz for t, _, wz in self._imu if t0 <= t <= t1]
        return float(np.mean(rates)) if rates else 0.0

    def height_at(self, t: float) -> float | None:
        """Altitude at time t, or None when the newest one is stale."""
        if not self._heights or t - self._heights[-1][0] > HEIGHT_STALE:
            return None
        ts, hs = zip(*self._heights, strict=True)
        return float(np.interp(t, ts, hs))

    def camera_height(
        self,
        altitude: float,
        r_world_body: NDArray[np.float64],
    ) -> float:
        """Camera above the floor [m].

        Nose-down lowers the camera, since it sits ahead of the body
        origin.
        """
        return (
            altitude + self._height_landed + r_world_body[2, 0] * self._ahead
        )

    def image_cb(self, msg: CompressedImage) -> None:
        """Turn one camera frame into a velocity over the floor."""
        t = stamp_to_sec(msg) - self._delay
        if not self._imu or t - self._imu[-1][0] > IMU_STALE:
            self._prev = None
            return
        altitude = self.height_at(t)
        if altitude is None:
            self._prev = None
            return
        gray = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_REDUCED_GRAYSCALE_2,
        )
        if gray is None:
            self.get_logger().warning("Could not decode a camera frame.")
            self._prev = None
            return
        r_world_body = self.attitude_at(t)
        height = self.camera_height(altitude, r_world_body)
        self._heartbeat.publish(Empty())

        prev = self._prev
        self._prev = (t, gray, r_world_body, height)
        if prev is None or t <= prev[0]:
            return
        dt = t - prev[0]
        step = self.track(prev[1:], (gray, r_world_body, height))
        if step is None:
            return
        d_fwd, d_left, n, spread = step
        d_left -= self.mean_yaw_rate(t - dt, t) * self._ahead * dt
        step = (d_fwd, d_left, n, spread)
        if self.check(height, step, dt):
            return
        sigma = math.hypot(
            SIGMA_MIN,
            SIGMA_SPREAD_FACTOR * spread / math.sqrt(n) / dt,
        )
        self.publish(t, d_fwd / dt, d_left / dt, sigma)

    def floor_points(
        self,
        px: NDArray,
        r_world_body: NDArray[np.float64],
        height: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.bool_]]:
        """Floor points (x, y) [m] seen at pixels px, and which hit it.

        px are half-size pixels. Each is undistorted into a ray out of
        the camera, turned into world axes through the mount and the
        attitude, and scaled until it has gone down by the camera
        height [m].
        """
        n = cv2.fisheye.undistortPoints(
            (px * DECODE_SCALE).reshape(-1, 1, 2).astype(np.float64),
            self._k,
            self._d,
        ).reshape(-1, 2)
        rays = (
            np.c_[n, np.ones(len(n))] @ (r_world_body @ self._camera_to_body).T
        )
        hits = rays[:, 2] < -RAY_MIN_DOWN
        scale = -height / np.where(hits, rays[:, 2], -1.0)
        return rays[:, :2] * scale[:, None], hits

    def track(
        self,
        old: tuple[NDArray, NDArray[np.float64], float],
        new: tuple[NDArray, NDArray[np.float64], float],
    ) -> tuple[float, float, int, float] | None:
        """Drone displacement between two frames.

        old and new are (half-size gray image, body-to-world rotation,
        camera height [m]). Each track end is projected with its own
        frame's attitude and height. Returns (forward, left) [m] in the
        new frame's heading, the number of tracks, and the spread [m]:
        the median distance of the tracks from the result. None when
        there are too few tracks.
        """
        old_gray, old_r, old_h = old
        gray, r_world_body, height = new
        if self._mask is None:
            self._mask = np.zeros_like(gray)
            r0, r1 = self._rows
            self._mask[
                r0 // DECODE_SCALE : r1 // DECODE_SCALE,
                MASK_MARGIN:-MASK_MARGIN,
            ] = 255
        p0 = cv2.goodFeaturesToTrack(
            old_gray,
            self._max_corners,
            CORNER_QUALITY,
            CORNER_MIN_DISTANCE,
            mask=self._mask,
        )
        if p0 is None:
            return None
        p1, st, _ = cv2.calcOpticalFlowPyrLK(
            old_gray,
            gray,
            p0,
            None,  # type: ignore[call-overload]
            winSize=LK_WIN_SIZE,
            maxLevel=LK_MAX_LEVEL,
            criteria=LK_CRITERIA,
        )
        pb, stb, _ = cv2.calcOpticalFlowPyrLK(
            gray,
            old_gray,
            p1,
            None,  # type: ignore[call-overload]
            winSize=LK_WIN_SIZE,
            maxLevel=LK_MAX_LEVEL,
            criteria=LK_CRITERIA,
        )
        fb_error = np.linalg.norm((pb - p0)[:, 0], axis=1)
        good = (st[:, 0] == 1) & (stb[:, 0] == 1) & (fb_error < FB_MAX_ERROR)
        if np.count_nonzero(good) < MIN_VECTORS:
            return None
        g0, hit0 = self.floor_points(p0[good, 0], old_r, old_h)
        g1, hit1 = self.floor_points(p1[good, 0], r_world_body, height)
        hit = hit0 & hit1
        n = int(hit.sum())
        if n < MIN_VECTORS:
            return None
        dv = g1[hit] - g0[hit]
        dw = -np.median(dv, axis=0)
        spread = float(np.median(np.linalg.norm(dv + dw, axis=1)))
        yaw = math.atan2(r_world_body[1, 0], r_world_body[0, 0])
        d_fwd = math.cos(yaw) * dw[0] + math.sin(yaw) * dw[1]
        d_left = -math.sin(yaw) * dw[0] + math.cos(yaw) * dw[1]
        return d_fwd, d_left, n, spread

    def check(
        self,
        height: float,
        step: tuple[float, float, int, float],
        dt: float,
    ) -> str:
        """Name of the first failed frame check, or "" when all pass.

        step is (forward, left, tracks, spread) as track returns it.
        """
        d_fwd, d_left, n, spread = step
        d = math.hypot(d_fwd, d_left)
        if height < GATE_H_MIN:
            return "low"
        if n < GATE_N_MIN:
            return "corners"
        if spread > GATE_SPREAD_STEP * d + GATE_SPREAD_HEIGHT * height:
            return "spread"
        if d > GATE_V_MAX * dt:
            return "speed"
        return ""

    def publish(
        self,
        t: float,
        v_fwd: float,
        v_left: float,
        sigma: float,
    ) -> None:
        """Publish the velocity (forward, left) [m/s] stamped at t [s].

        Only x and y are measured; the other covariances are -1.
        """
        msg = TwistWithCovarianceStamped()
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t % 1) * 1e9)
        msg.header.frame_id = self._frame_id
        msg.twist.twist.linear.x = v_fwd
        msg.twist.twist.linear.y = v_left
        cov = [0.0] * 36
        cov[0] = cov[7] = sigma**2
        cov[14] = cov[21] = cov[28] = cov[35] = -1.0
        msg.twist.covariance = cov
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Run the optical flow node."""
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
