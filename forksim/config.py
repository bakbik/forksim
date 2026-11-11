"""Every tunable number of the simulation lives here.

World frame: a room with its origin in the bottom-left corner, x to the right,
y up, angles in degrees counter-clockwise from +x.

Two groups matter for the experiments:

* ``PerceptionErrors`` - how wrong the robot's picture of the pallet is.
* ``ControlErrors``    - how differently the truck moves from what it was told.

``ideal()`` sets both groups to zero (the simulator that "almost always works").
``nominal()`` is a guess at a real truck. The nominal values are ASSUMED, not
measured: pinning them down is exactly what the real-forklift measurements in
the README are for.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, replace


@dataclass(frozen=True)
class Room:
    width: float = 20.0    # x extent, m
    height: float = 30.0   # y extent, m


@dataclass(frozen=True)
class Pallet:
    """Static pallet (EUR by default). Its pose is the pose of its centre; the
    angle is the direction the forks travel through it.

    It has four edges: two ENTRY edges (the `width` sides, with fork pockets) and
    two SIDE edges (the `length` sides, solid). Simplification: the three rows of
    blocks are modelled as three solid stringers running the full length.
    """
    x: float = 10.0
    y: float = 15.1
    angle_deg: float = 90.0
    length: float = 1.20       # size x: along the fork direction
    width: float = 0.80        # size y: across the entry edge
    height: float = 0.144      # size z - stored, not used by the 2D model
    weight_kg: float = 400.0   # stored, not used: the pallet is static and lifting is not simulated
    outer_block: float = 0.10
    centre_block: float = 0.145

    @property
    def depth(self) -> float:
        return self.length

    @property
    def blocks(self) -> tuple:
        """Lateral extent (y_min, y_max) of each stringer, pallet frame."""
        h, c = 0.5 * self.width, 0.5 * self.centre_block
        return ((-h, -h + self.outer_block), (-c, c), (h - self.outer_block, h))


@dataclass(frozen=True)
class Forklift:
    """SEER SFL-CBD15 autonomous pallet truck. Values marked (spec) are from the
    manufacturer's data sheet; the rest are derived from it or assumed.

    It is a pallet truck, not a counterbalance truck: the forks ride on load
    wheels near their tips and the single drive wheel under the power unit both
    drives and steers. Reference point = centre of the load-wheel axle (the
    point the truck pivots about). Heading points towards the fork tips.
    """
    x: float = 10.0               # start pose, set by the user
    y: float = 8.5
    angle_deg: float = 90.0
    max_speed: float = 2.0        # m/s (spec: 2 m/s; CE version 1.4)
    max_steer_deg: float = 90.0   # derived: the 1524 mm turning radius means it pivots about the load axle
    min_turn_radius: float = 1.524  # m (spec). Tightest curve it may follow while driving into a pallet;
                                  # sets how much approach error the align phase can still absorb.
    max_accel: float = 0.8        # assumed
    rated_load_kg: float = 1500.0  # (spec)
    weight_kg: float = 388.0      # (spec) with battery - stored, not used
    lift_height: float = 0.205    # (spec) - stored, lifting is not simulated
    length: float = 1.712         # (spec) overall, fork tip to rear of power unit
    width: float = 0.932          # (spec)
    fork_length: float = 1.22     # (spec)
    fork_width: float = 0.17      # (spec)
    fork_outer_width: float = 0.55  # (spec) 0.55 / 0.60 / 0.68 available; 0.55 fits a EUR pallet's 800 mm edge
    load_wheel_setback: float = 0.26  # derived: fork tip to load axle, chosen so the swept radius is 1524 mm
    wheelbase: float = 1.20       # assumed: load axle to drive wheel

    @property
    def reach(self) -> float:
        """Reference point to fork tip."""
        return self.load_wheel_setback

    @property
    def fork_heel(self) -> float:
        """Carriage face (the truck's contact edge) relative to the reference point; negative = behind it."""
        return self.load_wheel_setback - self.fork_length

    @property
    def fork_spacing(self) -> float:
        """Fork centre to fork centre."""
        return self.fork_outer_width - self.fork_width

    @property
    def body_rear(self) -> float:
        """Reference point to the rear of the power unit."""
        return self.length - self.load_wheel_setback


@dataclass(frozen=True)
class LidarSpec:
    """2D scanner at pallet-block height, mounted at the carriage between the forks.
    (spec: scanning range up to 360 deg. Mount position and the numbers below are assumed;
    the body blocking the rear part of the view is not modelled.)"""
    fov_deg: float = 360.0
    resolution_deg: float = 0.25
    max_range: float = 25.0
    rate_hz: float = 10.0


@dataclass(frozen=True)
class CameraSpec:
    """Coarse pallet detector. DISABLED for now: the truck runs on lidar alone.
    When enabled it only hints where a pallet might be while the lidar has no
    entry edge yet; it never sets the fork target."""
    enabled: bool = False
    fov_deg: float = 70.0
    max_range: float = 25.0
    dropout: float = 0.05


@dataclass(frozen=True)
class PerceptionErrors:
    # Range noise grows with distance: sigma = lidar_range_sigma + lidar_range_sigma_per_m * range.
    # The growing term also stands in for the beam spot widening and blurring edges. At 1x this is
    # 12 mm at 2 m, 15 mm at 3 m, 30 mm at 8 m - so a closer look is a better look.
    lidar_range_sigma: float = 0.006         # m, noise right at the sensor
    lidar_range_sigma_per_m: float = 0.003   # m of extra noise per metre of range
    lidar_dropout: float = 0.02              # probability a beam returns nothing
    lidar_yaw_bias_sigma_deg: float = 0.5    # extrinsic calibration error, constant per trial
    lidar_lateral_bias_sigma: float = 0.005  # m, extrinsic calibration error, constant per trial
    time_sync_error: float = 0.06            # s, scan is older than the pose it is fused with
    camera_pos_sigma: float = 0.08           # m at 4 m range, grows linearly with range (camera is disabled by default)

    def scaled(self, s: float) -> "PerceptionErrors":
        return replace(self, **{f.name: getattr(self, f.name) * s for f in fields(self)})


@dataclass(frozen=True)
class ControlErrors:
    steer_bias_sigma_deg: float = 1.0   # steering zero offset, constant per trial
    steer_lag: float = 0.25             # s, first-order steering actuator
    steer_rate_limit_deg: float = 40.0  # deg/s. Scales inversely: 2x error = half the rate, 0x = no limit
    cmd_latency: float = 0.10           # s, pure delay on commands
    speed_lag: float = 0.40             # s, first-order drive response

    _SCALED = ("steer_bias_sigma_deg", "steer_lag", "cmd_latency", "speed_lag")

    def scaled(self, s: float) -> "ControlErrors":
        out = replace(self, **{k: getattr(self, k) * s for k in self._SCALED})
        return replace(out, steer_rate_limit_deg=self.steer_rate_limit_deg / s if s > 0 else math.inf)


@dataclass(frozen=True)
class Variation:
    """Trial-to-trial variation around the user-set poses. All uniform +/-."""
    pallet_pos: float = 0.15          # m
    pallet_angle_deg: float = 20.0    # wide enough that a side (wrong) edge is in view too
    start_lateral: float = 0.4        # m, across the truck's start heading
    start_heading_deg: float = 8.0    # approach-angle variation


@dataclass(frozen=True)
class TaskSpec:
    approach_distance: float = 1.5   # D: fork tips to the selected entry edge at the approach target
    handover_margin: float = 0.5     # share of the kinematically possible approach error we accept
    contact_tol: float = 0.005   # m of fork/block overlap tolerated (chamfer, compliance)
    timeout: float = 90.0


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.02
    room: Room = field(default_factory=Room)
    pallet: Pallet = field(default_factory=Pallet)
    forklift: Forklift = field(default_factory=Forklift)
    lidar: LidarSpec = field(default_factory=LidarSpec)
    camera: CameraSpec = field(default_factory=CameraSpec)
    perception: PerceptionErrors = field(default_factory=PerceptionErrors)
    control: ControlErrors = field(default_factory=ControlErrors)
    variation: Variation = field(default_factory=Variation)
    task: TaskSpec = field(default_factory=TaskSpec)


def nominal() -> SimConfig:
    return SimConfig()


def scaled(perception_scale: float, control_scale: float, base: SimConfig | None = None) -> SimConfig:
    """Scale every perception error and every control error relative to nominal.
    (0, 0) is the ideal simulator, (1, 1) is nominal."""
    base = base or nominal()
    return replace(base,
                   perception=base.perception.scaled(perception_scale),
                   control=base.control.scaled(control_scale))


def ideal() -> SimConfig:
    return scaled(0.0, 0.0)


def override(cfg: SimConfig, changes: dict) -> SimConfig:
    """override(cfg, {"perception.lidar_range_sigma": 0.0, "pallet.x": 12.0, "dt": 0.01})"""
    for path, value in changes.items():
        if "." in path:
            group, name = path.split(".", 1)
            cfg = replace(cfg, **{group: replace(getattr(cfg, group), **{name: value})})
        else:
            cfg = replace(cfg, **{path: value})
    return cfg


# ------------------------------------------------------------------ what counts as valid input
MIN_FORK_CLEARANCE = 0.010   # m a fork must have on each side inside its pocket, nominally


def limits(cfg: SimConfig) -> dict:
    """Allowed (min, max) for every property a user may set. Several follow from other
    properties: a pallet must be wide enough for these forks, light enough for this
    truck, and inside this room."""
    f, pl, room = cfg.forklift, cfg.pallet, cfg.room
    edge = 0.5 * math.hypot(pl.length, pl.width) + 0.3            # keep the whole pallet off the walls
    return {
        "room.width": (5.0, 100.0),
        "room.height": (5.0, 100.0),
        "pallet.x": (edge, room.width - edge),
        "pallet.y": (edge, room.height - edge),
        "pallet.angle_deg": (-360.0, 360.0),
        "pallet.length": (0.6, 2.5),
        # the two forks, at their outer width, must fit between the outer blocks with clearance
        "pallet.width": (round(f.fork_outer_width + 2.0 * (pl.outer_block + MIN_FORK_CLEARANCE), 3), 2.5),
        "pallet.height": (0.10, 0.25),
        "pallet.weight_kg": (1.0, f.rated_load_kg),
        "forklift.x": (1.0, room.width - 1.0),
        "forklift.y": (1.0, room.height - 1.0),
        "forklift.angle_deg": (-360.0, 360.0),
        "forklift.max_speed": (0.1, 2.0),
        "forklift.max_steer_deg": (10.0, 90.0),
        "forklift.min_turn_radius": (0.0, 5.0),
        "task.approach_distance": (0.3, 5.0),
        "task.handover_margin": (0.1, 1.0),
    }


def validate(cfg: SimConfig) -> list:
    """Everything wrong with a configuration, in words. Empty list = valid."""
    problems = []
    for path, (lo, hi) in limits(cfg).items():
        group, name = path.split(".")
        value = getattr(getattr(cfg, group), name)
        if not lo <= value <= hi:
            problems.append(f"{path} = {value:g} is outside the allowed range {lo:g} to {hi:g}")
    f, pl = cfg.forklift, cfg.pallet
    apart = 0.5 * math.hypot(pl.length, pl.width) + f.length
    if math.hypot(f.x - pl.x, f.y - pl.y) < apart:
        problems.append(f"the truck starts on top of the pallet: keep their positions at least {apart:.2f} m apart")
    return problems
