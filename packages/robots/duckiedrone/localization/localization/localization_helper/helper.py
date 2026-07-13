"""Monte-Carlo localization helper code for Duckiedrone."""

import math
from typing import List, Optional

import cv2
import numpy as np

ORB_GRID_SIZE_X = 4
ORB_GRID_SIZE_Y = 3
MATCH_RATIO = 0.7
MIN_MATCH_COUNT = 10
MAP_GRID_SIZE_X = ORB_GRID_SIZE_X * 3
MAP_GRID_SIZE_Y = ORB_GRID_SIZE_Y * 3
PROB_THRESHOLD = 0.001
ORB_FEATURES_PER_GRID_CELL = 500


class Particle:
    def __init__(self, i, poses, weights):
        self.i = i
        self.poses = poses
        self.weights = weights

    def weight(self):
        return self.weights[self.i]

    def x(self):
        return self.poses[self.i, 0]

    def y(self):
        return self.poses[self.i, 1]

    def z(self):
        return self.poses[self.i, 2]

    def yaw(self):
        return self.poses[self.i, 3]


class ParticleSet:
    def __init__(self, num_particles, poses):
        self.weights = np.full(num_particles, PROB_THRESHOLD)
        self.particles = [Particle(i, poses, self.weights) for i in range(num_particles)]
        self.poses = poses
        self.num_particles = num_particles


class LocalizationParticleFilter:
    def __init__(
        self,
        camera_width: int,
        camera_height: int,
        camera_center: np.ndarray,
        camera_scale=290.0,
        map_pixel_width=3227,
        map_pixel_height=2447,
        map_width_meters=1.4,
        map_height_meters=1.07,
        map_image_path: str = "map.jpg",
    ):
        self.map_kp: List[List[List[cv2.KeyPoint]]]
        self.map_des: List[List[List[cv2.typing.MatLike]]]

        self.particles: Optional[ParticleSet] = None
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        self.map_pixel_width = map_pixel_width
        self.map_pixel_height = map_pixel_height
        self.map_width_meters = map_width_meters
        self.map_height_meters = map_height_meters

        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_center = camera_center
        self.camera_scale = camera_scale
        self.meter_to_pixel = (
            float(self.map_pixel_width) / self.map_width_meters
            + float(self.map_pixel_height) / self.map_height_meters
        ) / 2.0

        self.keyframe_dist_threshold = min(self.camera_width, self.camera_height) - 40
        self.keyframe_yaw_threshold = 0.175
        self.cell_x = float(self.map_pixel_width) / MAP_GRID_SIZE_X
        self.cell_y = float(self.map_pixel_height) / MAP_GRID_SIZE_Y

        self.create_map(map_image_path)
        self.key_kp = None
        self.key_des = None
        self.z = 0.0
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.sigma_x = 0.05
        self.sigma_y = 0.05
        self.sigma_yaw = 0.01
        sigma_vx = 0.01
        sigma_vy = 0.01
        sigma_vz = 0.0
        sigma_yaw = 0.01
        self.covariance_motion = np.array(
            [
                [sigma_vx**2, 0, 0, 0],
                [0, sigma_vy**2, 0, 0],
                [0, 0, sigma_vz**2, 0],
                [0, 0, 0, sigma_yaw**2],
            ]
        )

    def update(self, z, angle_x, angle_y, prev_kp, prev_des, kp, des):
        self.z = z
        self.angle_x = angle_x
        self.angle_y = angle_y

        transform = self.compute_transform(prev_kp, prev_des, kp, des)
        if transform is not None:
            x = self.pixel_to_meter(-transform[0, 2])
            y = self.pixel_to_meter(transform[1, 2])
            yaw = np.arctan2(transform[1, 0], transform[0, 0])
            self.sample_motion_model(x, y, yaw)

            if self.key_kp is not None and self.key_des is not None:
                transform = self.compute_transform(self.key_kp, self.key_des, kp, des)
                if transform is not None:
                    x = -transform[0, 2]
                    y = transform[1, 2]
                    yaw = -np.arctan2(transform[1, 0], transform[0, 0])
                    if distance(x, y, 0, 0) > self.keyframe_dist_threshold or yaw > self.keyframe_yaw_threshold:
                        self.measurement_model(kp, des)
                        self.key_kp, self.key_des = kp, des
                else:
                    self.measurement_model(kp, des)
                    self.key_kp, self.key_des = kp, des
            else:
                self.measurement_model(kp, des)
                self.key_kp, self.key_des = kp, des

        self.resample_particles()
        return self.get_estimated_position()

    def sample_motion_model(self, x, y, yaw):
        noisy_x_y_z_yaw = np.random.multivariate_normal([x, y, self.z, yaw], self.covariance_motion)
        for i in range(self.particles.num_particles):
            pose = self.particles.poses[i]
            old_yaw = pose[3]
            pose[0] += noisy_x_y_z_yaw[0] * np.cos(old_yaw) - noisy_x_y_z_yaw[1] * np.sin(old_yaw)
            pose[1] += noisy_x_y_z_yaw[0] * np.sin(old_yaw) + noisy_x_y_z_yaw[1] * np.cos(old_yaw)
            pose[2] = self.z
            pose[3] = adjust_angle(pose[3] + noisy_x_y_z_yaw[3])

    def measurement_model(self, kp, des):
        for i in range(self.particles.num_particles):
            position = self.particles.poses[i]
            grid_x = int(position[0] * self.meter_to_pixel / self.cell_x)
            grid_x = max(min(grid_x, MAP_GRID_SIZE_X - 1), 0)
            grid_y = int(position[1] * self.meter_to_pixel / self.cell_y)
            grid_y = max(min(grid_y, MAP_GRID_SIZE_Y - 1), 0)
            sub_map_kp = self.map_kp[grid_x][grid_y]
            sub_map_des = self.map_des[grid_x][grid_y]
            pose, _ = self.compute_location(kp, des, sub_map_kp, sub_map_des)

            if pose is None:
                q = PROB_THRESHOLD
            else:
                noisy_pose = [
                    np.random.normal(pose[0], self.sigma_x),
                    np.random.normal(pose[1], self.sigma_y),
                    pose[2],
                    np.random.normal(pose[3], self.sigma_yaw),
                ]
                noisy_pose[3] = adjust_angle(noisy_pose[3])
                yaw_difference = adjust_angle(noisy_pose[3] - position[3])
                q = (
                    norm_pdf(noisy_pose[0] - position[0], 0, self.sigma_x)
                    * norm_pdf(noisy_pose[1] - position[1], 0, self.sigma_y)
                    * norm_pdf(yaw_difference, 0, self.sigma_yaw)
                )

            self.particles.weights[i] = max(q, PROB_THRESHOLD)

    def resample_particles(self):
        weights_sum = np.sum(self.particles.weights)
        new_poses = []
        new_weights = []
        normal_weights = self.particles.weights / float(weights_sum)
        samples = np.random.multinomial(self.particles.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                new_poses.append(self.particles.poses[i])
                new_weights.append(self.particles.weights[i])
        self.particles.poses = np.array(new_poses)
        self.particles.weights = np.array(new_weights)

    def get_estimated_position(self):
        weights_sum = np.sum(self.particles.weights)
        x = 0.0
        y = 0.0
        z = 0.0
        yaw = 0.0
        normal_weights = self.particles.weights / float(weights_sum)
        for i, prob in enumerate(normal_weights):
            x += prob * self.particles.poses[i, 0]
            y += prob * self.particles.poses[i, 1]
            z += prob * self.particles.poses[i, 2]
            yaw += prob * self.particles.poses[i, 3]
        return Particle(0, np.array([[x, y, z, yaw]]), np.array([weights_sum / self.particles.weights.size]))

    def initialize_particles(self, num_particles, kp, des):
        self.key_kp, self.key_des = None, None
        weights_sum = 0.0
        weights = []
        poses = []
        new_poses = []

        for x in range(MAP_GRID_SIZE_X):
            for y in range(MAP_GRID_SIZE_Y):
                p, w = self.compute_location(kp, des, self.map_kp[x][y], self.map_des[x][y])
                if p is not None:
                    poses.append([p[0], p[1], p[2], p[3]])
                    weights.append(w)
                    weights_sum += w

        if len(poses) == 0:
            for x in range(MAP_GRID_SIZE_X):
                for y in range(MAP_GRID_SIZE_Y):
                    poses.append(
                        [
                            (x * self.cell_x + self.cell_x / 2.0) / self.meter_to_pixel,
                            (y * self.cell_y + self.cell_y / 2.0) / self.meter_to_pixel,
                            self.z,
                            np.random.random_sample() * 2 * np.pi - np.pi,
                        ]
                    )
                    weights_sum += 1.0
                    weights.append(1.0)

        weights = np.array(weights) / weights_sum
        samples = np.random.multinomial(num_particles, weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                new_poses.append(poses[i])

        self.particles = ParticleSet(num_particles, np.array(new_poses))
        return self.get_estimated_position()

    def compute_location(self, kp1, des1, kp2, des2):
        good = []
        pose = None
        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)
            for match in matches:
                if len(match) > 1 and match[0].distance < MATCH_RATIO * match[1].distance:
                    good.append(match[0])

            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.array([kp1[m.queryIdx].pt for m in good], dtype=np.float32).reshape(-1, 1, 2)
                dst_pts = np.array([kp2[m.trainIdx].pt for m in good], np.float32).reshape(-1, 1, 2)
                transform, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
                if transform is not None:
                    transformed_center = cv2.transform(self.camera_center, transform)
                    transformed_center = [
                        transformed_center[0][0][0] / self.meter_to_pixel,
                        (self.map_pixel_height - 1 - transformed_center[0][0][1]) / self.meter_to_pixel,
                    ]
                    yaw = np.arctan2(transform[1, 0], transform[0, 0])
                    z = math.sqrt(self.z**2 / (1 + math.tan(self.angle_x) ** 2 + math.tan(self.angle_y) ** 2))
                    offset_x = np.tan(self.angle_x) * z
                    offset_y = np.tan(self.angle_y) * z
                    global_offset_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
                    global_offset_y = math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
                    pose = [
                        transformed_center[0] + global_offset_x,
                        transformed_center[1] + global_offset_y,
                        z,
                        yaw,
                    ]
        return pose, len(good)

    def compute_transform(self, kp1, des1, kp2, des2):
        transform = None
        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)
            good = []
            for match in matches:
                if len(match) > 1 and match[0].distance < MATCH_RATIO * match[1].distance:
                    good.append(match[0])
            src_pts = np.array([kp1[m.queryIdx].pt for m in good], dtype=np.float32).reshape(-1, 1, 2)
            dst_pts = np.array([kp2[m.trainIdx].pt for m in good], dtype=np.float32).reshape(-1, 1, 2)
            if src_pts is not None and dst_pts is not None and len(src_pts) > 3 and len(dst_pts) > 3:
                transform, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
        return transform

    def pixel_to_meter(self, px):
        return px * self.z / self.camera_scale

    def create_map(self, file_name: str):
        image = cv2.imread(file_name)
        max_total_keypoints = ORB_FEATURES_PER_GRID_CELL * ORB_GRID_SIZE_X * ORB_GRID_SIZE_Y
        detector = cv2.ORB.create(nfeatures=max_total_keypoints, scoreType=cv2.ORB_FAST_SCORE)
        kp = detector.detect(image, None)
        kp, des = detector.compute(image, kp)

        grid_kp = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
        grid_des = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
        for i in range(len(kp)):
            x = int(kp[i].pt[0] / self.cell_x)
            y = MAP_GRID_SIZE_Y - 1 - int(kp[i].pt[1] / self.cell_y)
            grid_kp[x][y].append(kp[i])
            grid_des[x][y].append(des[i])

        map_grid_kp = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
        map_grid_des = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
        for i in range(MAP_GRID_SIZE_X):
            for j in range(MAP_GRID_SIZE_Y):
                for k in range(-1, 2):
                    for l in range(-1, 2):
                        x = i + k
                        y = j + l
                        if 0 <= x < MAP_GRID_SIZE_X and 0 <= y < MAP_GRID_SIZE_Y:
                            map_grid_kp[i][j].extend(grid_kp[x][y])
                            map_grid_des[i][j].extend(grid_des[x][y])
                map_grid_des[i][j] = np.array(map_grid_des[i][j])

        self.map_kp = map_grid_kp
        self.map_des = map_grid_des


def norm_pdf(x, mu, sigma):
    u = (x - mu) / float(abs(sigma))
    return (1 / (np.sqrt(2 * np.pi) * abs(sigma))) * np.exp(-u * u / 2.0)


def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


def adjust_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle