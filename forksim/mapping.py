"""Local map: lidar returns cached in the odometry frame.

The truck knows nothing about the room in advance. All it has is the current
scan plus whatever it has seen before. New data always wins: every cached point
that the new scan looked at, or looked through, is thrown away before the new
returns are added. A cached point therefore survives only where the scanner
cannot currently see (outside its field of view, or hidden behind something),
so if the world changed, the map follows as soon as the change is in view.
"""
from __future__ import annotations

import math

import numpy as np


class LocalMap:
    def __init__(self, angles: np.ndarray, max_age: float = 30.0, max_points: int = 4000,
                 clear_margin: float = 0.15):
        self.angles = angles
        self._half = float(angles[-1])
        self._res = float(angles[1] - angles[0])
        self.max_age, self.max_points, self.clear_margin = max_age, max_points, clear_margin
        self.xy = np.empty((0, 2))      # odometry frame
        self.stamp = np.empty(0)
        self.n_fresh = 0                # how many of the points came from the latest scan

    def update(self, ranges: np.ndarray, sensor_pose, t: float) -> None:
        """Insert one scan taken from `sensor_pose` (odometry frame)."""
        sx, sy, sth = sensor_pose
        if self.stamp.size:
            dx, dy = self.xy[:, 0] - sx, self.xy[:, 1] - sy
            r = np.hypot(dx, dy)
            bearing = (np.arctan2(dy, dx) - sth + math.pi) % (2.0 * math.pi) - math.pi
            in_fov = np.abs(bearing) <= self._half
            beam = np.clip(np.rint((bearing + self._half) / self._res).astype(int), 0, ranges.size - 1)
            measured = ranges[beam]
            # the beam reached at least this far, so whatever was cached here is out of date
            superseded = in_fov & np.isfinite(measured) & (r < measured + self.clear_margin)
            keep = ~superseded & (t - self.stamp < self.max_age)
            self.xy, self.stamp = self.xy[keep], self.stamp[keep]
        ok = np.isfinite(ranges)
        a = self.angles[ok] + sth
        fresh = np.column_stack((sx + ranges[ok] * np.cos(a), sy + ranges[ok] * np.sin(a)))
        self.n_fresh = fresh.shape[0]
        self.xy = np.vstack((self.xy, fresh))[-self.max_points:]
        self.stamp = np.concatenate((self.stamp, np.full(fresh.shape[0], t)))[-self.max_points:]

    def in_frame(self, pose, max_dist: float = math.inf):
        """Map points expressed in the frame of `pose` (odometry frame), as (x, y) arrays."""
        dx, dy = self.xy[:, 0] - pose[0], self.xy[:, 1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        px, py = c * dx + s * dy, -s * dx + c * dy
        if math.isfinite(max_dist):
            near = px * px + py * py < max_dist * max_dist
            px, py = px[near], py[near]
        return px, py
