"""One pallet pickup attempt: world + truck + lidar + map + perception + behaviour.

`TrialRunner` is the gym-style seam (construct = reset, `step()` = one 20 ms
tick). `run_trial()` runs it to the end and returns a `TrialResult`.

How a trial ends:

    success     the pallet touched the truck's carriage face with both forks in
                their pockets - the one contact that means "loaded"
    collision   any other contact between truck and pallet. The naive collision
                check stops the simulation on the spot and says what hit what.
    a flag      the behaviour itself declined to continue (see behaviors.FLAGS)
    left_room / timeout

Alignment is expressed by two measurable numbers, recorded as ground truth at
three moments (handover from approach to align, fork tips crossing the entry
edge, end of trial), next to the robot's own estimate at handover:

    lateral  offset between the pallet centre line and the fork centre line,
             taken at the entry-edge plane
    angle    between the pallet's entry edge and the truck's carriage face
             (= truck heading relative to the pallet axis)

Seeding: (base_seed, trial) fully determines a trial. Two independent random
streams are derived from it - scenario and sensor noise - and the
scenario stream always draws the same variates in the same order, so trial k
has the same pallet, start pose and per-trial biases (up to scale) for every
controller and every config. Comparisons are therefore paired.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass

import numpy as np

from .behaviors import FLAGS, BehaviorParams, PickupBehavior
from .config import SimConfig, validate
from .geometry import compose, rect_segments, relative, wrap_half
from .mapping import LocalMap
from .perception import LidarPalletEstimator, OraclePerception
from .sensors import Camera, Lidar2D
from .vehicle import ForkliftDynamics

COLLISIONS = ("collision_fork_face",   # a fork tip ran into the front of a block: lateral miss
              "collision_fork_wall",   # a fork scraped a block from inside its pocket: angular miss
              "collision_side",        # the truck drove at a side (wrong) edge
              "collision_body")        # the power unit hit the pallet
OUTCOMES = ("success", *COLLISIONS, *FLAGS, "left_room", "timeout")
NAN = float("nan")


@dataclass
class TrialResult:
    trial: int
    controller: str
    success: bool              # pallet loaded: carriage face on the entry edge, forks in the pockets
    outcome: str               # one of OUTCOMES
    phase: str                 # behaviour phase the trial ended in
    det_pos_mm: float          # error of the last pallet detection: centre position ...
    det_angle_deg: float       # ... and axis angle
    handover_lat_mm: float     # truth: the positioning error approach handed to align
    handover_angle_deg: float
    est_lat_mm: float          # the same two as the robot estimated them at that moment
    est_angle_deg: float
    entry_lat_mm: float        # truth when the fork tips crossed the entry edge
    entry_angle_deg: float
    load_lat_mm: float         # truth when the trial ended (loaded or collided)
    load_angle_deg: float
    bad_detections: int        # detections that had the pallet turned 90 deg (side taken for entry)
    final_depth_m: float       # fork tip depth past the entry edge at the end
    max_pen_mm: float          # worst fork/block overlap seen
    duration_s: float

    def as_dict(self) -> dict:
        return asdict(self)


def _outline(x0: float, x1: float, y0: float, y1: float, step: float):
    """Sample points along the front edge (x = x1) and along the rest of a rectangle's outline."""
    ys = np.arange(y0, y1 + 1e-9, step)
    xs = np.arange(x0, x1, step)
    front = np.column_stack((np.full(ys.size, x1), ys))
    rest = np.vstack((np.column_stack((np.full(ys.size, x0), ys)),
                      np.column_stack((xs, np.full(xs.size, y0))), np.column_stack((xs, np.full(xs.size, y1)))))
    return front, rest


class TrialRunner:
    def __init__(self, cfg: SimConfig, controller: str = "actuator_aware", trial: int = 0,
                 base_seed: int = 0, manual: bool = False, perception: str = "lidar",
                 ctrl_params: BehaviorParams | None = None):
        problems = validate(cfg)
        if problems:
            raise ValueError("invalid configuration: " + "; ".join(problems))
        self.cfg, self.trial, self.manual = cfg, trial, manual
        streams = np.random.SeedSequence([base_seed, trial]).spawn(2)
        r_scn, self._r_sens = (np.random.default_rng(s) for s in streams)
        u = r_scn.uniform(-1.0, 1.0, 5)
        z = r_scn.standard_normal(4)

        va, vg, pl, pe, ce = cfg.variation, cfg.forklift, cfg.pallet, cfg.perception, cfg.control
        self.pallet_centre = (pl.x + u[0] * va.pallet_pos, pl.y + u[1] * va.pallet_pos,
                              math.radians(pl.angle_deg + u[2] * va.pallet_angle_deg))
        # pallet frame = one entry edge, x axis into the pallet; the opposite edge is the other entry
        self.pallet_pose = compose(self.pallet_centre, (-0.5 * pl.depth, 0.0, 0.0))
        self.entry_frames = (self.pallet_pose, compose(self.pallet_pose, (pl.depth, 0.0, math.pi)))
        start = compose((vg.x, vg.y, math.radians(vg.angle_deg)),
                        (0.0, u[3] * va.start_lateral, math.radians(u[4] * va.start_heading_deg)))

        segs = []
        for y0, y1 in pl.blocks:
            segs += rect_segments(0.0, pl.depth, y0, y1, self.pallet_pose)
        segs += rect_segments(0.0, cfg.room.width, 0.0, cfg.room.height, (0.0, 0.0, 0.0))   # room walls
        self.segs = np.array(segs)

        self.vehicle = ForkliftDynamics(vg, ce, cfg.dt, start,
                                        steer_bias=math.radians(z[0] * ce.steer_bias_sigma_deg))
        self.mount = (vg.fork_heel, 0.0, 0.0)                      # where perception believes the lidar is
        mount_true = (vg.fork_heel, z[3] * pe.lidar_lateral_bias_sigma,
                      math.radians(z[2] * pe.lidar_yaw_bias_sigma_deg))
        self.lidar = Lidar2D(cfg.lidar, pe, mount_true)
        self.map = LocalMap(self.lidar.angles)
        self.camera = Camera(cfg.camera, pe, self.mount) if cfg.camera.enabled else None
        if perception == "oracle":
            self.estimator = OraclePerception(0.01, 0.5, self._r_sens)
        else:
            self.estimator = LidarPalletEstimator(pl, sensor_xy=self.mount[:2])
        self.ctrl = PickupBehavior(controller, vg, pl, cfg.task, ctrl_params, self.map)

        self._lidar_every = max(1, int(round(1.0 / (cfg.lidar.rate_hz * cfg.dt))))
        self._hist = deque([start], maxlen=int(round(pe.time_sync_error / cfg.dt)) + 1)
        self._k = 0
        self.t = 0.0
        self.done = False
        self.outcome = None
        self.alert = ""             # what the collision check saw, in words
        self.depth = NAN
        self.max_pen = 0.0
        self.handover = None        # (lat, angle, est_lat, est_angle) when align began
        self.entry = None           # (lat, angle) when the fork tips crossed the entry edge
        self.det_err = (NAN, NAN)
        self.bad_detections = 0
        self.ranges = None          # latest scan, for display
        self.scan_pose = start

        # outline sample points in the truck frame, for the naive collision check
        tip = []
        for yc in (0.5 * vg.fork_spacing, -0.5 * vg.fork_spacing):
            front, rest = _outline(vg.fork_heel, vg.reach, yc - 0.5 * vg.fork_width, yc + 0.5 * vg.fork_width, 0.02)
            tip += [front, rest]
        self._fork_pts = np.vstack(tip)
        self._face_pts, self._body_pts = _outline(-vg.body_rear, vg.fork_heel, -0.5 * vg.width, 0.5 * vg.width, 0.03)

    # ------------------------------------------------------------------ step
    def step(self, manual_action=None) -> bool:
        if self.done:
            return True
        veh = self.vehicle
        if self._k % self._lidar_every == 0 and self.ctrl.wants_perception:
            self._perceive()
        if self.manual:
            v_cmd, s_cmd = manual_action or (0.0, 0.0)
            finished = False
        else:
            v_cmd, s_cmd, finished = self.ctrl.act(self.t, veh.odom, veh.v, veh.steer)
        veh.step(v_cmd, s_cmd)
        self._hist.append(veh.pose)
        self._k += 1
        self.t = self._k * self.cfg.dt
        self._check(finished)
        return self.done

    def _facing_entry(self, pose):
        """The true entry edge a truck at `pose` points into, the truck expressed relative to
        that edge, and whether it is really facing a side edge instead."""
        rels = [relative(f, pose) for f in self.entry_frames]
        rel = min(rels, key=lambda r: abs(r[2]))
        return rel, abs(rel[2]) > math.radians(45.0)

    def true_params(self):
        """Ground-truth (lateral mm, angle deg) right now; NaN if the truck faces a side edge."""
        rel, side = self._facing_entry(self.vehicle.pose)
        if side:
            return (NAN, NAN)
        return ((rel[1] - rel[0] * math.tan(rel[2])) * 1e3, math.degrees(rel[2]))

    def _perceive(self) -> None:
        veh = self.vehicle
        # time-sync error: the scan was taken a little earlier than the pose it gets fused with
        self.scan_pose = self._hist[0]
        self.ranges = self.lidar.scan(self.scan_pose, self.segs, self._r_sens)
        # new returns go into the map, replacing whatever was cached where the scan looked
        self.map.update(self.ranges, compose(veh.odom, self.mount), self.t)
        px, py = self.map.in_frame(veh.odom, self.cfg.lidar.max_range)
        expect = relative(veh.odom, self.ctrl.pallet) if self.ctrl.pallet is not None else None
        truth = relative(veh.pose, self.pallet_centre)       # true pallet pose seen from the truck
        if isinstance(self.estimator, OraclePerception):
            self.estimator.truth_robot = truth
        det = self.estimator.detect(px, py, expect, self.ctrl.entry_only)
        b = self.estimator.blob
        if b is None and self.camera is not None and expect is None:
            b = self.camera.detect(veh.pose, self.pallet_centre, self._r_sens)   # optional hint, off by default
        if b is not None:
            self.ctrl.observe_coarse(compose(veh.odom, (b[0], b[1], 0.0))[:2])
        if det is None:
            return
        err = relative(truth, det.pallet)
        if abs(wrap_half(err[2])) > math.radians(45.0):
            self.bad_detections += 1
        else:
            self.det_err = (math.hypot(err[0], err[1]) * 1e3, math.degrees(wrap_half(err[2])))
        from_range = math.hypot(det.pallet[0] - self.mount[0], det.pallet[1] - self.mount[1])
        self.ctrl.observe(compose(veh.odom, det.pallet), self.t, from_range)

    # ------------------------------------------------------- collision check
    @staticmethod
    def _to_frame(rel, pts):
        c, s = math.cos(rel[2]), math.sin(rel[2])
        return rel[0] + c * pts[:, 0] - s * pts[:, 1], rel[1] + s * pts[:, 0] + c * pts[:, 1]

    def _fork_overlap(self, rel) -> float:
        """Deepest overlap between the forks and the pallet blocks, metres."""
        X, Y = self._to_frame(rel, self._fork_pts)
        inside = (X > 0.0) & (X < self.cfg.pallet.depth)
        if not inside.any():
            return 0.0
        X, Y = X[inside], Y[inside]
        Xd = np.minimum(X, self.cfg.pallet.depth - X)
        pen = 0.0
        for y0, y1 in self.cfg.pallet.blocks:
            hit = (Y > y0) & (Y < y1)
            if hit.any():
                pen = max(pen, float(np.minimum(np.minimum(Y[hit] - y0, y1 - Y[hit]), Xd[hit]).max()))
        return pen

    def _in_footprint(self, rel, pts, tol: float) -> bool:
        """Does any of these truck points lie inside the pallet's footprint by more than tol?"""
        X, Y = self._to_frame(rel, pts)
        half = 0.5 * self.cfg.pallet.width
        return bool(((X > tol) & (X < self.cfg.pallet.depth - tol) & (np.abs(Y) < half - tol)).any())

    def _forks_in_pockets(self, rel) -> bool:
        b = self.cfg.pallet.blocks
        half = 0.5 * self.cfg.forklift.fork_spacing
        left = compose(rel, (self.cfg.forklift.reach, half, 0.0))[1]
        right = compose(rel, (self.cfg.forklift.reach, -half, 0.0))[1]
        return b[1][1] < left < b[2][0] and b[0][1] < right < b[1][0]

    def _check(self, finished: bool) -> None:
        veh, task, vg = self.vehicle, self.cfg.task, self.cfg.forklift
        rel, side = self._facing_entry(veh.pose)      # truck relative to the entry edge it is heading into
        self.depth = NAN if side else rel[0] + vg.reach * math.cos(rel[2])
        if self.handover is None and self.ctrl.handover is not None:
            self.handover = (*self.true_params(), *self.ctrl.handover)

        # naive collision detection: any truck/pallet contact stops the simulation. Exactly one
        # kind of contact is not a collision - the carriage face meeting the entry edge with both
        # forks in their pockets - and that one means the pallet is loaded.
        if math.hypot(veh.x - self.pallet_centre[0], veh.y - self.pallet_centre[1]) < vg.length + 1.0:
            if not side and self.entry is None and self.depth >= 0.0:
                self.entry = self.true_params()
            pen = self._fork_overlap(rel)
            self.max_pen = max(self.max_pen, pen)
            if pen > task.contact_tol:
                if side:
                    return self._collide("collision_side", "fork hit a side edge of the pallet")
                if self.depth < 0.08:
                    return self._collide("collision_fork_face", "fork tip hit the front of a block")
                return self._collide("collision_fork_wall", "fork scraped a block inside its pocket")
            if self._in_footprint(rel, self._body_pts, task.contact_tol):
                return self._collide("collision_body", "truck body hit the pallet")
            X, Y = self._to_frame(rel, self._face_pts)
            if bool(((X >= 0.0) & (X < self.cfg.pallet.depth) & (np.abs(Y) < 0.5 * self.cfg.pallet.width)).any()):
                if side or not self._forks_in_pockets(rel):
                    return self._collide("collision_side" if side else "collision_body",
                                         "truck body hit the pallet")
                self.ctrl.notify_loaded()
                return self._finish("success")
        if finished and self.ctrl.flag:
            self.alert = f"behaviour raised flag: {self.ctrl.flag}"
            return self._finish(self.ctrl.flag)
        if not (0.0 < veh.x < self.cfg.room.width and 0.0 < veh.y < self.cfg.room.height):
            return self._finish("left_room")
        if self.t >= task.timeout and not self.manual:
            self._finish("timeout")

    def _collide(self, outcome: str, what: str) -> None:
        self.alert = f"COLLISION: {what}"
        self._finish(outcome)

    def _finish(self, outcome: str) -> None:
        self.done, self.outcome = True, outcome

    # ---------------------------------------------------------------- result
    def result(self) -> TrialResult:
        h = self.handover or (NAN, NAN, NAN, NAN)
        e = self.entry or (NAN, NAN)
        end = self.true_params() if self.entry is not None else (NAN, NAN)
        return TrialResult(trial=self.trial, controller=self.ctrl.name,
                           success=self.outcome == "success", outcome=self.outcome or "running",
                           phase=self.ctrl.phase, det_pos_mm=self.det_err[0], det_angle_deg=self.det_err[1],
                           handover_lat_mm=h[0], handover_angle_deg=h[1], est_lat_mm=h[2], est_angle_deg=h[3],
                           entry_lat_mm=e[0], entry_angle_deg=e[1], load_lat_mm=end[0], load_angle_deg=end[1],
                           bad_detections=self.bad_detections,
                           final_depth_m=self.depth, max_pen_mm=self.max_pen * 1e3, duration_s=self.t)


def run_trial(cfg: SimConfig, controller: str, trial: int, base_seed: int = 0,
              perception: str = "lidar", ctrl_params: BehaviorParams | None = None) -> TrialResult:
    runner = TrialRunner(cfg, controller, trial, base_seed, perception=perception, ctrl_params=ctrl_params)
    while not runner.step():
        pass
    return runner.result()
