# ruff: noqa: ANN001,ANN201,ANN204,COM812,D101,D102,D103,D107,E501,NPY002,PERF401,PLR0913,PLR2004,W292
"""Support utilities for the Duckiedrone FastSLAM port."""

import math
import sys

import cv2
import numpy as np
from numpy import dot, identity

max_float = sys.float_info.max
MATCH_RATIO = 0.7
debug = False


class Landmark:
    def __init__(self, x, y, covariance, des, count):
        self.x = x
        self.y = y
        self.covariance = covariance
        self.des = des
        self.counter = count


def calculate_jacobian(robot_position, landmark_pos):
    t = robot_position
    j = landmark_pos
    q = (j[0] - t[0]) ** 2 + (j[1] - t[1]) ** 2
    return np.array(
        [
            [(j[0] - t[0]) / math.sqrt(q), (j[1] - t[1]) / math.sqrt(q)],
            [-(j[1] - t[1]) / q, (j[0] - t[0]) / q],
        ]
    )


def compute_measurement_covariance(jacobian, old_covariance, sigma_observation):
    return dot(dot(jacobian, old_covariance), np.transpose(jacobian)) + sigma_observation


def compute_initial_covariance(jacobian, sigma_observation):
    jacobian_inverse = np.linalg.inv(jacobian)
    return dot(dot(jacobian_inverse, sigma_observation), np.transpose(jacobian_inverse))


def compute_kalman_gain(jacobian, old_covariance, measurement_covariance):
    return dot(dot(old_covariance, np.transpose(jacobian)), np.linalg.inv(measurement_covariance))


def compute_new_landmark(z, z_hat, kalman_gain, old_landmark):
    z = np.array(z)
    z_hat = np.array(z_hat)
    d = z - z_hat
    d[1] = d[1] % (math.pi * 2)
    return tuple(old_landmark + dot(kalman_gain, d))


def compute_new_covariance(kalman_gain, jacobian, old_covariance):
    difference = identity(2) - dot(kalman_gain, jacobian)
    return dot(difference, old_covariance)


def add_landmark(particle, kp, des, sigma_observation, kp_to_measurement):
    robot_x, robot_y = particle.pose[0], particle.pose[1]
    dist, bearing = kp_to_measurement(kp)
    land_x = robot_x + (dist * np.cos(bearing))
    land_y = robot_y + (dist * np.sin(bearing))
    h_jacobian = calculate_jacobian((robot_x, robot_y), (land_x, land_y))
    covariance = compute_initial_covariance(h_jacobian, sigma_observation)
    particle.landmarks.append(Landmark(land_x, land_y, covariance, des, 1))


def update_landmark(particle, landmark, kp, des, sigma_observation, kp_to_measurement):
    robot_x, robot_y = particle.pose[0], particle.pose[1]
    dist, bearing = kp_to_measurement(kp)
    predicted_dist = distance(landmark.x, landmark.y, robot_x, robot_y)
    predicted_bearing = math.atan2((landmark.y - robot_y), (landmark.x - robot_x))
    covariance = landmark.covariance
    h_jacobian = calculate_jacobian((robot_x, robot_y), (landmark.x, landmark.y))
    measurement_covariance = compute_measurement_covariance(h_jacobian, covariance, sigma_observation)
    kalman_gain = compute_kalman_gain(h_jacobian, covariance, measurement_covariance)
    old_landmark = np.array([landmark.x, landmark.y])
    new_landmark = compute_new_landmark((dist, bearing), (predicted_dist, predicted_bearing), kalman_gain, old_landmark)
    new_covariance = compute_new_covariance(kalman_gain, h_jacobian, covariance)
    return Landmark(new_landmark[0], new_landmark[1], new_covariance, des, landmark.counter)


def compute_transform(matcher, kp1, des1, kp2, des2):
    transform = None
    if des1 is not None and des2 is not None:
        matches = matcher.knnMatch(des1, des2, k=2)
        good = []
        for match in matches:
            if len(match) > 1 and match[0].distance < MATCH_RATIO * match[1].distance:
                good.append(match[0])
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        if src_pts is not None and dst_pts is not None and len(src_pts) > 3 and len(dst_pts) > 3:
            transform, _ = cv2.estimateAffinePartial2D(src_pts, dst_pts)
    return transform


def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


def normal(mu, sigma):
    return np.random.normal(mu, sigma)


def adjust_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


class ThreadQueue:
    def __init__(self):
        self.queue = []

    def add_thread(self, thread):
        if len(self.queue) == 1:
            old_thread = self.queue[0]
            if not old_thread.is_alive():
                self.queue.remove(old_thread)
                self.queue.append(thread)
                thread.start()
        elif len(self.queue) == 0:
            self.queue.append(thread)
            thread.start()