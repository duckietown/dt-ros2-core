"""
Lane filter with the `dt-core` (ente, ROS 1) behaviour, built on top of the `dt_state_estimation`
library implementation.

The library version differs from the filter used by the ROS 1 lane-following demo in two ways that
change the estimate:

- it does not filter segments by distance (the `range_est` check is commented out), so far-away
  segments vote as much as near ones;
- its vote formula for white left-edge and yellow segments is an older one.

This subclass restores both from `dt-core/packages/lane_filter/include/lane_filter/lane_filter.py`.
"""
from math import sqrt
from typing import List

import numpy as np

from dt_state_estimation.lane_filter import LaneFilterHistogram as _LibraryLaneFilterHistogram
from dt_state_estimation.lane_filter.types import Segment, SegmentColor


class LaneFilterHistogram(_LibraryLaneFilterHistogram):
    """Histogram lane filter with the dt-core (ente) segment selection and vote generation."""

    def _prepare_segments(self, segments: List[Segment]) -> List[Segment]:
        filtered_segments = []
        for segment in segments:
            # optional transform from RED to WHITE
            if self.red_to_white and segment.color == SegmentColor.RED:
                segment.color = SegmentColor.WHITE
            # optional filtering out YELLOW
            if not self.use_yellow and segment.color == SegmentColor.YELLOW:
                continue
            # we don't care about RED ones for now
            if segment.color != SegmentColor.WHITE and segment.color != SegmentColor.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            # only consider points in a certain range from the robot for the position estimation
            point_range = self._get_segment_distance(segment)
            if self.range_est > point_range > self.range_est_min:
                filtered_segments.append(segment)
        return filtered_segments

    def _generate_vote(self, segment: Segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)

        n_hat = np.array([-t_hat[1], t_hat[0]])
        d1 = np.inner(n_hat, p1)
        d2 = np.inner(n_hat, p2)
        l1 = np.inner(t_hat, p1)
        l2 = np.inner(t_hat, p2)
        if l1 < 0:
            l1 = -l1
        if l2 < 0:
            l2 = -l2

        d_i = (d1 + d2) / 2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == SegmentColor.WHITE:  # right lane is white
            if p1[0] > p2[0]:  # right edge of white lane
                d_i -= self.linewidth_white
            else:  # left edge of white lane
                d_i -= self.linewidth_white
                d_i = self.lanewidth * 2 + self.linewidth_yellow - d_i
                phi_i = -phi_i
            d_i -= self.lanewidth / 2

        elif segment.color == SegmentColor.YELLOW:  # left lane is yellow
            if p2[0] > p1[0]:  # left edge of yellow lane
                d_i -= self.linewidth_yellow
                d_i = self.lanewidth / 2 - d_i
                phi_i = -phi_i
            else:  # right edge of yellow lane
                d_i += self.linewidth_yellow
                d_i -= self.lanewidth / 2

        return d_i, phi_i

    @staticmethod
    def _get_segment_distance(segment: Segment) -> float:
        x_c = (segment.points[0].x + segment.points[1].x) / 2
        y_c = (segment.points[0].y + segment.points[1].y) / 2
        return sqrt(x_c**2 + y_c**2)
