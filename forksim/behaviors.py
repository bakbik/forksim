"""The truck's behaviours, in order. Perception stays in the loop the whole way.

    detect    wait for the lidar pipeline to confirm a pallet. Output: the pallet's
              position and angle relative to the robot (noisy - it is a lidar fit).
    approach  from that, work out the approach pose: a fixed distance D in front of
              the centre of one of the two entry edges, facing it squarely. Get
              there with the most trivial trajectory this truck can do, because it
              pivots about its load axle: turn on the spot, drive straight, turn on
              the spot. Ends with some positioning error.
    align     take that error as lateral offset (pallet centre line vs fork centre
              line) and yaw (edge vs carriage face) and close both while driving
              in, until the pallet touches the truck body. That contact is the
              pallet being loaded: success.

Any phase can raise a flag instead of handing over to the next one:

    no_pallet_detected        nothing confirmed within the time allowed
    no_approach_position      neither entry edge has a usable approach pose
    approach_error_too_large  arrived too far off for the alignment to close
    lost_pallet               tracking lost during alignment

Two parameter sets share this code:

    baseline        tuned in the ideal simulator, where steering is instant
    actuator_aware  the same behaviour, but it knows the steering has delay, lag
                    and a rate limit: it predicts across the delay, and it does not
                    drive until the steered wheel has caught up with its command
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .config import Forklift, Pallet, TaskSpec
from .geometry import compose, relative, wrap, wrap_half

PHASES = ("detect", "approach", "align", "loaded", "flagged")
FLAGS = ("no_pallet_detected", "no_approach_position", "approach_error_too_large", "lost_pallet")


def acceptable_angle(distance: float, radius: float, lateral: float) -> float:
    """Largest yaw error (rad) at the approach target that the align phase can still close.

    The truck has `distance` (D) left before the pallet and may not turn tighter
    than `radius` (R). Worst case, to end up square on the centre line it must

      1. straighten up: an arc through the yaw error psi, which uses R*sin(psi) of
         the distance and carries it a further R*(1 - cos(psi)) sideways;
      2. remove the sideways offset it now has with an S-curve (two equal arcs),
         which over a length x shifts it by at most 2R*(1 - sqrt(1 - (x/2R)^2)).

    So a small D forces tiny errors and a large D leaves room to settle. Returns 0
    when the lateral offset alone is already more than D allows.
    """
    if radius <= 0.0:
        return 0.5 * math.pi

    def shift(x: float) -> float:
        x = max(0.0, min(x, 2.0 * radius))
        return 2.0 * radius * (1.0 - math.sqrt(1.0 - (x / (2.0 * radius)) ** 2))

    def closable(psi: float) -> bool:
        used = radius * math.sin(psi)
        return used <= distance and abs(lateral) + radius * (1.0 - math.cos(psi)) <= shift(distance - used)

    if not closable(0.0):
        return 0.0
    lo, hi = 0.0, 0.5 * math.pi
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if closable(mid) else (lo, mid)
    return lo


@dataclass(frozen=True)
class BehaviorParams:
    # detect
    init_scans: int = 5               # consistent detections needed to confirm a pallet
    detect_timeout: float = 15.0      # s without a confirmed pallet -> flag
    forget: float = 0.6               # how much of the accumulated confidence survives each new detection
    range_trust: float = 0.5          # 1/m: lidar noise grows with range, so a detection from range r
                                      # is weighted 1 / (1 + range_trust * r)^2. Closer looks count for more.
    # approach
    clearance: float = 0.75           # m that must be free of lidar returns around an approach position
    v_approach: float = 0.8           # m/s on the straight leg (capped by the truck's max_speed)
    v_pivot: float = 0.3              # m/s drive-wheel speed while turning on the spot
    k_drive: float = 2.0              # heading hold on the straight leg
    turn_tol_deg: float = 1.5         # pivot accuracy before driving off
    face_tol_deg: float = 0.5         # pivot accuracy at the approach pose
    # align
    v_slow: float = 0.25              # m/s while driving in
    lookahead: float = 0.6            # m, line-of-sight guidance distance
    k_heading: float = 5.0            # steer = k * heading error
    last_look_range: float = 0.35     # m: closer than this the scanner can no longer frame the edge
    stale_after: float = 1.0          # s without a valid detection before the truck holds still
    lost_timeout: float = 5.0         # s without a valid detection -> flag
    # actuator awareness - all off in the baseline
    delay_comp: float = 0.0           # s, predict the pose this far ahead before computing commands
    steer_wait_deg: float = math.inf  # hold back while |steer command - steer measured| is this large


CONTROLLERS = {
    "baseline": BehaviorParams(),
    "actuator_aware": BehaviorParams(delay_comp=0.25, steer_wait_deg=8.0),
}


class PickupBehavior:
    def __init__(self, name: str, geom: Forklift, pallet: Pallet, task: TaskSpec,
                 params: BehaviorParams | None = None, local_map=None):
        self.name, self.g, self.pl, self.task, self.map = name, geom, pallet, task, local_map
        self.p = params or CONTROLLERS[name]
        self.phase = "detect"
        self.step_name = "search"  # finer-grained label for display
        self.flag = None
        self.pallet = None         # tracked pallet pose (centre, axis angle), odometry frame
        self.coarse = None         # nearest pallet-sized thing seen so far, odometry frame
        self.locked = False        # True once the scanner is at the edge: no more looks
        self.lat_est = math.nan    # the robot's own view of the two alignment errors
        self.angle_est = math.nan
        self.handover = None       # (lat_est mm, angle_est deg) handed from approach to align
        self.angle_limit = math.nan  # deg: acceptance limit on the yaw error at the approach target
        self._edge_end = 0         # which entry edge was chosen: -1 or +1 along the pallet axis
        self._first = []
        self._info = 0.0           # accumulated confidence in the tracked pallet pose
        self._last_obs_t = 0.0
        self._sub = "turn"

    # ---------------------------------------------------------------- inputs
    @property
    def wants_perception(self) -> bool:
        return not self.locked and self.phase not in ("loaded", "flagged")

    @property
    def entry_only(self) -> bool:
        """During alignment only detections made from the entry edge itself are trusted."""
        return self.phase == "align"

    def observe_coarse(self, xy_odom) -> None:
        self.coarse = xy_odom

    def observe(self, pallet_odom, t: float, range_m: float = 0.0) -> None:
        """Feed one pallet detection (centre and axis angle, odometry frame) made from `range_m` away.
        The axis angle is only meaningful modulo 180 deg: the pallet is symmetric."""
        if self.locked:
            return
        self._last_obs_t = t
        if self.pallet is None:
            # confirm: several detections in a row that agree with each other
            if self._first and (abs(wrap_half(pallet_odom[2] - self._first[0][2])) > math.radians(15.0)
                                or math.hypot(pallet_odom[0] - self._first[0][0], pallet_odom[1] - self._first[0][1]) > 0.3):
                self._first = []
            self._first.append(pallet_odom)
            if len(self._first) == self.p.init_scans:
                n, a0 = len(self._first), self._first[0][2]
                self.pallet = (sum(e[0] for e in self._first) / n, sum(e[1] for e in self._first) / n,
                               wrap(a0 + sum(wrap_half(e[2] - a0) for e in self._first) / n))
                self._info = self._weight(range_m) / (1.0 - self.p.forget)
            return
        # Blend by confidence. Each detection is weighted by how far away it was made; old
        # confidence fades (odometry drifts). At a constant range this is a plain moving average
        # with weight 1 - forget; while closing in, the newer, closer looks dominate.
        w = self._weight(range_m)
        self._info = self.p.forget * self._info + w
        a = w / self._info
        x, y, th = self.pallet
        self.pallet = (x + a * (pallet_odom[0] - x), y + a * (pallet_odom[1] - y),
                       wrap(th + a * wrap_half(pallet_odom[2] - th)))

    def _weight(self, range_m: float) -> float:
        return 1.0 / (1.0 + self.p.range_trust * range_m) ** 2

    def notify_loaded(self) -> None:
        """The pallet is against the carriage (contact switch)."""
        self.phase, self.step_name = "loaded", "loaded"

    # ---------------------------------------------------------------- geometry
    def edge(self):
        """The chosen entry edge: pose at its centre, x axis pointing into the pallet (odometry frame)."""
        half = 0.5 * self.pl.depth
        if self._edge_end < 0:
            return compose(self.pallet, (-half, 0.0, 0.0))
        return compose(self.pallet, (half, 0.0, math.pi))

    def approach_pose(self):
        """Where the reference point must be so the fork tips are D in front of the edge, facing it."""
        return compose(self.edge(), (-(self.task.approach_distance + self.g.reach), 0.0, 0.0))

    def _choose_edge(self, odom) -> bool:
        """Pick the nearer entry edge whose approach pose is usable. False if neither is."""
        options = []
        for end in (-1, 1):
            self._edge_end = end
            ax, ay, _ = self.approach_pose()
            # free of lidar returns around the approach position? (returns from the pallet itself
            # do not count: with a small D the approach position is meant to be close to it)
            if self.map is not None and self.map.xy.size:
                mx, my = self.map.xy[:, 0], self.map.xy[:, 1]
                own = (mx - self.pallet[0]) ** 2 + (my - self.pallet[1]) ** 2 < (0.5 * math.hypot(self.pl.depth, self.pl.width) + 0.25) ** 2
                d2 = ((mx - ax) ** 2 + (my - ay) ** 2)[~own]
                if d2.size and float(d2.min()) < self.p.clearance ** 2:
                    continue
            # the straight leg must not cut through the pallet itself
            px, py = self.pallet[0] - odom[0], self.pallet[1] - odom[1]
            lx, ly = ax - odom[0], ay - odom[1]
            length2 = lx * lx + ly * ly
            s = max(0.0, min(1.0, (px * lx + py * ly) / length2)) if length2 > 1e-9 else 0.0
            keep_off = 0.5 * math.hypot(self.pl.depth, self.pl.width) + 0.5 * self.g.width + 0.1
            # ... but the approach position itself may be nearer than that (small D): then the rule
            # is only that the truck never gets closer on the way than it will be when it arrives
            keep_off = min(keep_off, math.hypot(px - lx, py - ly) - 0.05)
            if math.hypot(px - s * lx, py - s * ly) < keep_off:
                continue
            options.append((math.hypot(lx, ly), end))
        if not options:
            self._edge_end = 0
            return False
        self._edge_end = min(options)[1]
        return True

    # ---------------------------------------------------------------- output
    def _raise(self, flag: str):
        self.flag, self.phase, self.step_name = flag, "flagged", flag
        return 0.0, 0.0, True

    def _pivot(self, err: float, steer_meas: float):
        """Turn on the spot about the load axle: drive wheel at full lock, creep."""
        lock = math.radians(self.g.max_steer_deg)
        # if the wheel is already over to one side, keep it there and reverse the drive instead
        steer = math.copysign(lock, steer_meas if abs(steer_meas) > 0.5 * lock else err)
        v = min(self.p.v_pivot, max(0.01, 1.5 * self.g.wheelbase * abs(err)))
        if math.isfinite(self.p.steer_wait_deg) and abs(steer - steer_meas) > math.radians(5.0):
            v = 0.0                                      # wait for the wheel to get to full lock first
        return math.copysign(v, err * steer), steer, False

    def _hold_back(self, v_cmd: float, steer: float, steer_meas: float) -> float:
        if not math.isfinite(self.p.steer_wait_deg):
            return v_cmd
        lagging = abs(steer - steer_meas) / math.radians(self.p.steer_wait_deg)
        return max(0.05, v_cmd * max(0.0, 1.0 - lagging))

    def act(self, t: float, odom, v_meas: float, steer_meas: float):
        """Returns (drive-wheel speed command, steering command, finished)."""
        p, g = self.p, self.g
        stopped = abs(v_meas) < 0.01
        if self.phase in ("loaded", "flagged"):
            return 0.0, 0.0, True

        if self.phase == "detect":
            if self.pallet is None:
                if t > p.detect_timeout and (self.coarse is None or t - self._last_obs_t > p.detect_timeout):
                    return self._raise("no_pallet_detected")
                # Something pallet-sized in view but too far to make out its edges: close in on it.
                if self.coarse is None or t < 1.0 or self._first:
                    return 0.0, 0.0, False
                dx, dy = self.coarse[0] - odom[0], self.coarse[1] - odom[1]
                if math.hypot(dx, dy) < 4.0:
                    return 0.0, 0.0, False
                self.step_name = "closing in"
                return min(p.v_approach, g.max_speed), 2.0 * wrap(math.atan2(dy, dx) - odom[2]), False
            if not self._choose_edge(odom):
                return self._raise("no_approach_position")
            self.phase, self._sub = "approach", "turn"

        if p.delay_comp > 0.0:
            # where the truck will be by the time this command takes effect
            d = v_meas * math.cos(steer_meas) * p.delay_comp
            dth = v_meas * math.sin(steer_meas) / g.wheelbase * p.delay_comp
            odom = (odom[0] + d * math.cos(odom[2] + 0.5 * dth),
                    odom[1] + d * math.sin(odom[2] + 0.5 * dth), wrap(odom[2] + dth))
        xr, yr, thr = relative(self.edge(), odom)        # truck in the believed entry-edge frame
        self.lat_est = (yr - xr * math.tan(thr)) * 1e3 if abs(thr) < 1.4 else math.nan
        self.angle_est = math.degrees(thr)
        since_seen = t - self._last_obs_t

        if self.phase == "approach":
            goal = self.approach_pose()
            dx, dy = goal[0] - odom[0], goal[1] - odom[1]
            dist = math.hypot(dx, dy)
            bearing_err = wrap(math.atan2(dy, dx) - odom[2])
            if self._sub == "turn":
                self.step_name = "turn to approach point"
                if dist < 0.10:
                    self._sub = "face"
                elif abs(bearing_err) < math.radians(p.turn_tol_deg):
                    if stopped:
                        self._sub = "drive"
                    return 0.0, 0.0, False
                else:
                    return self._pivot(bearing_err, steer_meas)
            if self._sub == "drive":
                self.step_name = "drive to approach point"
                along = dx * math.cos(odom[2]) + dy * math.sin(odom[2])
                if along <= 0.01:
                    if stopped:
                        self._sub = "face"
                    return 0.0, 0.0, False
                if dist > 0.5 and abs(bearing_err) > math.radians(20.0):
                    self._sub = "turn"                   # the goal moved a lot (re-estimated): re-aim
                    return 0.0, 0.0, False
                steer = max(-0.45, min(0.45, p.k_drive * bearing_err)) if dist > 0.3 else 0.0
                v_cmd = min(min(p.v_approach, g.max_speed), max(0.05, 0.8 * along))
                return self._hold_back(v_cmd, steer, steer_meas), steer, False
            # face the entry edge squarely
            self.step_name = "turn to face edge"
            face_err = wrap(goal[2] - odom[2])
            if abs(face_err) >= math.radians(p.face_tol_deg):
                return self._pivot(face_err, steer_meas)
            if not stopped or abs(steer_meas) > math.radians(2.0):
                return 0.0, 0.0, False                   # settle, wheel back to straight
            # arrived: hand the positioning error over to alignment - if alignment can still close it.
            # The acceptance limit follows from D and the minimum turning radius (see acceptable_angle).
            self.handover = (self.lat_est, self.angle_est)
            lateral = abs(self.lat_est) * 1e-3 if self.lat_est == self.lat_est else math.inf
            self.angle_limit = math.degrees(self.task.handover_margin * acceptable_angle(
                self.task.approach_distance, g.min_turn_radius, lateral))
            if not abs(self.angle_est) <= self.angle_limit:
                return self._raise("approach_error_too_large")
            self.phase = "align"

        # ---- align: close lateral and yaw error while driving in, until contact
        self.step_name = "align and drive in"
        if not self.locked and xr + g.fork_heel * math.cos(thr) > -p.last_look_range:
            self.locked = True                           # scanner is right at the edge: finish blind
        lost = not self.locked and since_seen > p.stale_after
        if not self.locked and since_seen > p.lost_timeout:
            return self._raise("lost_pallet")
        heading_des = math.atan2(-yr, p.lookahead)       # line-of-sight guidance onto the edge's centre line
        # while driving in, never turn tighter than the minimum turning radius
        max_steer = min(math.radians(g.max_steer_deg), math.atan2(g.wheelbase, g.min_turn_radius))
        steer = max(-max_steer, min(max_steer, p.k_heading * wrap(heading_des - thr)))
        if lost:
            return 0.0, steer, False                     # hold still until the pallet is seen again
        to_contact = -(xr + g.fork_heel * math.cos(thr))  # believed carriage-face distance to the edge
        v_cmd = min(p.v_slow, g.max_speed) if to_contact > 0.25 else min(0.10, g.max_speed)
        return self._hold_back(v_cmd, steer, steer_meas), steer, False
