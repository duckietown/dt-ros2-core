from duckiematrix_engine.template import MatrixEntityBehavior

import copy
import numpy as np


class StraightLineScript(MatrixEntityBehavior):

    def __init__(self, *args, distance: float = 1.0, speed: float = 0.1):
        super(StraightLineScript, self).__init__(*args)
        self._initial_pose = copy.deepcopy(self.pose.as_frame()['pose'])
        self.pose.x = self._initial_pose['x']
        self.pose.y = self._initial_pose['y']
        self.pose.yaw = self._initial_pose['yaw']
        self._distance = distance
        self._distance_on_leg = 0
        self._speed = speed

    def update(self, delta_t: float):
        self.pose.x = self.pose.x - self._speed * delta_t * np.cos(self.pose.yaw)
        self.pose.y = self.pose.y - self._speed * delta_t * np.sin(self.pose.yaw)
        self._distance_on_leg += self._speed * delta_t
        if self._distance_on_leg > self._distance:
            self.pose.yaw += np.deg2rad(180)
            self._distance_on_leg = 0
        # update frames
        self.pose.commit()
