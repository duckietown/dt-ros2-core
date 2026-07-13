"""FastSLAM helper code for Duckiedrone."""

import copy
import math
import threading

import cv2
import numpy as np

from . import utils

CAMERA_SCALE = 290.0
CAMERA_WIDTH = 480.0
CAMERA_HEIGHT = 640.0
MATCH_RATIO = 0.7
PROB_THRESHOLD = 0.005
KEYFRAME_DIST_THRESHOLD = CAMERA_HEIGHT
KEYFRAME_YAW_THRESHOLD = 0.175
POSE = False
WEIGHT = False
pose_path = "/home/luke/ws/src/pidrone_pkg/scripts/pose_data.txt"


class Particle:
    def __init__(self, x, y, z, yaw):
        self.pose = [x, y, z, yaw]
        self.landmarks = []
        self.weight = PROB_THRESHOLD


class FastSLAM:
    def __init__(self):
        self.particles = None
        self.num_particles = None
        self.weight = PROB_THRESHOLD
        self.z = 0
        self.perceptual_range = 0.0
        self.key_kp = None
        self.key_des = None
        if POSE or WEIGHT:
            self.file = open(pose_path, "w")
        self.thread_queue = utils.ThreadQueue()
        self.most_recent_map = None
        self.new_result = False
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
        self.sigma_d = 3
        self.sigma_p = 0.30
        self.sigma_observation = np.array([[self.sigma_d**2, 0], [0, self.sigma_p**2]])
        sigma_vx, sigma_vy, sigma_vz, sigma_yaw = 2, 2, 0.0, 0.01
        self.covariance_motion = np.array(
            [[sigma_vx**2, 0, 0, 0], [0, sigma_vy**2, 0, 0], [0, 0, sigma_vz**2, 0], [0, 0, 0, sigma_yaw**2]]
        )

    def generate_particles(self, num_particles):
        self.particles = [Particle(abs(utils.normal(0, 0.1)), abs(utils.normal(0, 0.1)), self.z, abs(utils.normal(math.pi, 0.01))) for _ in range(num_particles)]
        self.num_particles = num_particles
        self.key_kp, self.key_des, self.most_recent_map = None, None, None
        self.new_result = False
        self.weight = PROB_THRESHOLD
        return self.estimate_pose()

    def run(self, z, prev_kp, prev_des, kp, des):
        self.z = z
        self.update_perceptual_range()
        transform = utils.compute_transform(self.matcher, prev_kp, prev_des, kp, des)
        if transform is not None:
            x = -transform[0, 2]
            y = transform[1, 2]
            yaw = -np.arctan2(transform[1, 0], transform[0, 0])
            for particle in self.particles:
                self.predict_particle(particle, x, y, yaw)
            self.detect_keyframe(kp, des)
            if self.new_result:
                self.new_result = False
                self.update_particles_from_map()
        return self.estimate_pose(), self.weight

    def predict_particle(self, particle, x, y, yaw):
        noisy_x_y_z_yaw = np.random.multivariate_normal([x, y, self.z, yaw], self.covariance_motion)
        particle.pose[0] += self.pixel_to_meter(noisy_x_y_z_yaw[0])
        particle.pose[1] += self.pixel_to_meter(noisy_x_y_z_yaw[1])
        particle.pose[2] = self.z
        particle.pose[3] = utils.adjust_angle(particle.pose[3] + noisy_x_y_z_yaw[3])

    def detect_keyframe(self, kp, des):
        if self.key_kp is not None and self.key_des is not None:
            transform = utils.compute_transform(self.matcher, self.key_kp, self.key_des, kp, des)
            if transform is not None:
                x = self.pixel_to_meter(-transform[0, 2])
                y = self.pixel_to_meter(transform[1, 2])
                yaw = -np.arctan2(transform[1, 0], transform[0, 0])
                if utils.distance(x, y, 0, 0) > self.pixel_to_meter(KEYFRAME_DIST_THRESHOLD) or yaw > KEYFRAME_YAW_THRESHOLD:
                    self.start_map_update_thread(kp, des)
            else:
                self.start_map_update_thread(kp, des)
        else:
            self.start_map_update_thread(kp, des)

    def start_map_update_thread(self, kp, des):
        thread = threading.Thread(target=self.update_map, args=(kp, des))
        self.thread_queue.add_thread(thread)
        self.key_kp, self.key_des = kp, des

    def update_map(self, kp, des):
        curr_particles = copy.deepcopy(self.particles)
        for particle in curr_particles:
            self.update_particle(particle, kp, des)
        self.most_recent_map = curr_particles
        self.new_result = True

    def update_particle(self, particle, keypoints, descriptors):
        particle.weight = PROB_THRESHOLD
        if len(particle.landmarks) == 0:
            for kp, des in zip(keypoints, descriptors):
                utils.add_landmark(particle, kp, des, self.sigma_observation, self.kp_to_measurement)
                particle.weight += math.log(PROB_THRESHOLD)
        else:
            close_landmarks = self.get_close_landmarks(particle)
            part_descriptors = [lm[0].des for lm in close_landmarks] if close_landmarks else None
            matched_landmarks = [False] * len(close_landmarks) if close_landmarks else None
            for kp, des in zip(keypoints, descriptors):
                match = self.matcher.knnMatch(np.array([des]), np.array(part_descriptors), k=2) if part_descriptors else None
                if not self.__is_valid_match(match):
                    utils.add_landmark(particle, kp, des, self.sigma_observation, self.kp_to_measurement)
                    particle.weight += math.log(PROB_THRESHOLD)
                else:
                    close_index = match[0].trainIdx
                    matched_landmarks[close_index] = True
                    lm = close_landmarks[close_index][0]
                    updated_landmark = utils.update_landmark(particle, lm, kp, des, self.sigma_observation, self.kp_to_measurement)
                    particle.landmarks[close_landmarks[close_index][1]] = updated_landmark
                    particle.weight += math.log(self.scale_weight(match[0].distance, match[1].distance))
            if matched_landmarks is not None:
                self.update_landmark_counters(particle, close_landmarks, matched_landmarks)

    def get_close_landmarks(self, particle):
        close_landmarks = []
        for index, landmark in enumerate(particle.landmarks):
            if utils.distance(landmark.x, landmark.y, particle.pose[0], particle.pose[1]) <= self.perceptual_range * 1.2:
                close_landmarks.append((landmark, index))
        return close_landmarks

    def __is_valid_match(self, match):
        return match is not None and len(match) >= 2 and match[0].distance < MATCH_RATIO * match[1].distance

    def update_landmark_counters(self, particle, close_landmarks, matched_landmarks):
        removed_landmarks = []
        for index, matched in enumerate(matched_landmarks):
            landmark, _particle_index = close_landmarks[index]
            if matched:
                landmark.counter += 1
            else:
                landmark.counter -= 1
                particle.weight += math.log(0.1 * PROB_THRESHOLD)
                if landmark.counter < 0:
                    removed_landmarks.append(landmark)
        for landmark in removed_landmarks:
            particle.landmarks.remove(landmark)

    def update_particles_from_map(self):
        most_recent_particles = self.most_recent_map
        for old_particle, new_particle in zip(self.particles, most_recent_particles):
            old_particle.landmarks = new_particle.landmarks
            old_particle.weight = new_particle.weight
        if WEIGHT:
            self.file.write(str([p.weight for p in self.particles]) + "\n")
        self.weight = self.get_average_weight()
        self.resample_particles()

    def resample_particles(self):
        weights = [p.weight for p in self.particles]
        lowest_weight = min(weights)
        normal_weights = np.array([1 - (w / lowest_weight) if w != 0 else PROB_THRESHOLD for w in weights])
        normal_weights /= np.sum(normal_weights)
        samples = np.random.multinomial(self.num_particles, normal_weights)
        new_particles = [copy.deepcopy(self.particles[i]) for i, count in enumerate(samples) for _ in range(count)]
        self.particles = new_particles

    def get_average_weight(self):
        return np.sum([p.weight for p in self.particles]) / float(self.num_particles)

    def pixel_to_meter(self, px):
        return px * self.z / CAMERA_SCALE

    def estimate_pose(self):
        weights = [p.weight for p in self.particles]
        lowest_weight = min(weights)
        normal_weights = np.array([1 - (w / lowest_weight) if w != 0 else PROB_THRESHOLD for w in weights])
        normal_weights /= np.sum(normal_weights)
        x = y = z = yaw = 0.0
        for index, prob in enumerate(normal_weights):
            x += prob * self.particles[index].pose[0]
            y += prob * self.particles[index].pose[1]
            z += prob * self.particles[index].pose[2]
            yaw += prob * self.particles[index].pose[3]
        return [x, y, z, utils.adjust_angle(yaw)]

    def update_perceptual_range(self):
        self.perceptual_range = self.pixel_to_meter(CAMERA_WIDTH / 2)

    def kp_to_measurement(self, kp):
        kp_x, kp_y = kp.pt[0], kp.pt[1]
        kp_y = CAMERA_HEIGHT - kp_y
        dx = kp_x - CAMERA_WIDTH / 2
        dy = kp_y - CAMERA_HEIGHT / 2
        dist = self.pixel_to_meter(math.sqrt(dx ** 2 + dy ** 2))
        bearing = math.atan2(dy, dx)
        return dist, bearing

    def scale_weight(self, match0, match1):
        scaled = (match1 - match0) / float(match1)
        return scaled if scaled != 0 else PROB_THRESHOLD