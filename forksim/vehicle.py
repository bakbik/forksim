"""Forklift motion model: kinematic bicycle plus the ways a real truck disobeys.

True motion and odometry are integrated side by side. They differ by the
steering zero offset, wheel-scale error and slip, none of which the encoders
see. That gap is "the robot does not drive exactly where it is told".

Pallet-truck layout: one wheel under the power unit both drives and steers, the
load wheels under the forks just roll. `v` is the speed of that drive wheel and
`steer` its angle (positive turns the truck left), so

    forward speed of the reference point = v * cos(steer)
    yaw rate                             = v * sin(steer) / wheelbase

At steer = 90 deg the truck pivots on the spot about the load axle.
"""
from __future__ import annotations

import math
from collections import deque

from .config import ControlErrors, Forklift
from .geometry import wrap


class ForkliftDynamics:
    def __init__(self, geom: Forklift, errs: ControlErrors, dt: float, pose,
                 steer_bias: float, speed_scale: float, rng):
        self.g, self.e, self.dt = geom, errs, dt
        self.x, self.y, self.th = pose          # truth
        self.ox, self.oy, self.oth = 0.0, 0.0, 0.0   # odometry: starts at zero, the truck has no idea where it is in the room
        self.v = 0.0                            # encoder speed
        self.steer = 0.0                        # encoder steering angle
        self.steer_bias = steer_bias
        self.speed_scale = speed_scale
        self._max_steer = math.radians(geom.max_steer_deg)
        self._steer_rate = math.radians(errs.steer_rate_limit_deg)
        self._slip = math.radians(errs.slip_sigma_deg)
        self._buf = deque([(0.0, 0.0)] * int(round(errs.cmd_latency / dt)))
        self._rng = rng
        self._noise = rng.standard_normal(4096)
        self._ni = 0

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    @property
    def odom(self):
        return (self.ox, self.oy, self.oth)

    def _normal(self) -> float:
        if self._ni >= len(self._noise):
            self._noise = self._rng.standard_normal(4096)
            self._ni = 0
        self._ni += 1
        return float(self._noise[self._ni - 1])

    def step(self, v_cmd: float, steer_cmd: float) -> None:
        dt, e = self.dt, self.e
        # pure delay between command and actuator
        self._buf.append((v_cmd, steer_cmd))
        v_cmd, steer_cmd = self._buf.popleft()
        v_cmd = max(-self.g.max_speed, min(self.g.max_speed, v_cmd))

        # steering actuator: first-order lag with a rate limit
        target = max(-self._max_steer, min(self._max_steer, steer_cmd))
        rate = (target - self.steer) / (e.steer_lag if e.steer_lag > 0 else dt)
        rate = max(-self._steer_rate, min(self._steer_rate, rate))
        self.steer += rate * dt

        # drive: first-order lag with an acceleration limit
        acc = (v_cmd - self.v) / (e.speed_lag if e.speed_lag > 0 else dt)
        acc = max(-self.g.max_accel, min(self.g.max_accel, acc))
        self.v += acc * dt

        # truth: the wheel is not where the encoder says, and the floor is not perfect
        v_true = self.v * self.speed_scale
        steer_true = self.steer + self.steer_bias
        ds = v_true * math.cos(steer_true) * dt
        dth = v_true * math.sin(steer_true) / self.g.wheelbase * dt
        dth += self._slip * math.sqrt(abs(ds)) * self._normal()
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th = wrap(self.th + dth)

        # odometry: integrates what the encoders report
        dth_o = self.v * math.sin(self.steer) / self.g.wheelbase * dt
        ds_o = self.v * math.cos(self.steer) * dt
        self.ox += ds_o * math.cos(self.oth + 0.5 * dth_o)
        self.oy += ds_o * math.sin(self.oth + 0.5 * dth_o)
        self.oth = wrap(self.oth + dth_o)
