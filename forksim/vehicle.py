"""Forklift motion model: kinematic pallet truck plus the ways a real one disobeys.

Pallet-truck layout: one wheel under the power unit both drives and steers, the
load wheels under the forks just roll. `v` is the speed of that drive wheel and
`steer` its angle (positive turns the truck left), so

    forward speed of the reference point = v * cos(steer)
    yaw rate                             = v * sin(steer) / wheelbase

At steer = 90 deg the truck pivots on the spot about the load axle.

True motion and odometry are integrated side by side. They differ by the
steering zero offset: the wheel is not quite where its encoder says, so the
truck turns a little when the robot believes it is going straight.

Wheel slip and wheel-radius error are neglected.
"""
from __future__ import annotations

import math
from collections import deque

from .config import ControlErrors, Forklift
from .geometry import wrap


class ForkliftDynamics:
    def __init__(self, geom: Forklift, errs: ControlErrors, dt: float, pose, steer_bias: float):
        self.g, self.e, self.dt = geom, errs, dt
        self.x, self.y, self.th = pose          # truth
        self.ox, self.oy, self.oth = 0.0, 0.0, 0.0   # odometry: starts at zero, the truck has no idea where it is in the room
        self.v = 0.0                            # drive wheel speed
        self.steer = 0.0                        # encoder steering angle
        self.steer_bias = steer_bias
        self._max_steer = math.radians(geom.max_steer_deg)
        self._steer_rate = math.radians(errs.steer_rate_limit_deg)
        self._buf = deque([(0.0, 0.0)] * int(round(errs.cmd_latency / dt)))

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    @property
    def odom(self):
        return (self.ox, self.oy, self.oth)

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

        # truth: the wheel is not where the encoder says
        steer_true = self.steer + self.steer_bias
        ds = self.v * math.cos(steer_true) * dt
        dth = self.v * math.sin(steer_true) / self.g.wheelbase * dt
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th = wrap(self.th + dth)

        # odometry: integrates what the encoders report
        dth_o = self.v * math.sin(self.steer) / self.g.wheelbase * dt
        ds_o = self.v * math.cos(self.steer) * dt
        self.ox += ds_o * math.cos(self.oth + 0.5 * dth_o)
        self.oy += ds_o * math.sin(self.oth + 0.5 * dth_o)
        self.oth = wrap(self.oth + dth_o)
