"""Sensor models. They produce raw measurements from ground truth; turning
measurements into a fork target is perception's job.

Lidar is the only sensor in use. The camera model is kept but disabled
(`CameraSpec.enabled = False`)."""
from __future__ import annotations

import math

import numpy as np

from .config import CameraSpec, LidarSpec, PerceptionErrors
from .geometry import compose, raycast, relative


class Lidar2D:
    """Planar scanner. `mount_true` is where it really sits on the truck;
    perception only knows the nominal mount, so any difference between the two
    is an extrinsic calibration error."""

    def __init__(self, spec: LidarSpec, errs: PerceptionErrors, mount_true):
        half = math.radians(spec.fov_deg) / 2.0
        n = int(round(spec.fov_deg / spec.resolution_deg)) + 1
        self.angles = np.linspace(-half, half, n)
        self.spec, self.errs, self.mount_true = spec, errs, mount_true

    def scan(self, robot_pose, segs: np.ndarray, rng) -> np.ndarray:
        sx, sy, sth = compose(robot_pose, self.mount_true)
        r = raycast((sx, sy), self.angles + sth, segs)
        r = r + self.errs.lidar_range_sigma * rng.standard_normal(r.size)
        lost = rng.random(r.size) < self.errs.lidar_dropout
        r[lost | (r > self.spec.max_range) | (r <= 0.0)] = np.nan
        return r


class Camera:
    """Stand-in for a camera pallet detector: a coarse position (x, y) of the
    pallet centre in the robot frame, or None. Disabled by default."""

    def __init__(self, spec: CameraSpec, errs: PerceptionErrors, mount):
        self.spec, self.errs, self.mount = spec, errs, mount

    def detect(self, robot_pose, pallet_centre, rng):
        z = rng.standard_normal(2)
        dropped = rng.random() < self.spec.dropout
        x, y, _ = relative(compose(robot_pose, self.mount), (pallet_centre[0], pallet_centre[1], 0.0))
        rng_m = math.hypot(x, y)
        if dropped or rng_m > self.spec.max_range or abs(math.atan2(y, x)) > math.radians(self.spec.fov_deg) / 2:
            return None
        sig = self.errs.camera_pos_sigma * max(rng_m, 1.0) / 4.0
        return compose(self.mount, (x + sig * z[0], y + sig * z[1], 0.0))[:2]
