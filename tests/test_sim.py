"""Fast checks of the properties the experiments rely on. Run: python -m pytest -q"""
import math
import re
from pathlib import Path

import numpy as np
import pytest

import forksim as fs
from forksim.behaviors import acceptable_angle
from forksim.geometry import compose, inverse, raycast, rect_segments, relative
from forksim.mapping import LocalMap
from forksim.perception import LidarPalletEstimator
from forksim.sensors import Lidar2D
from forksim.trial import TrialRunner

ROOT = Path(__file__).resolve().parents[1]


def test_pose_maths_round_trips():
    a, b = (1.0, -2.0, 0.7), (0.3, 0.4, -1.1)
    back = relative(a, compose(a, b))
    assert np.allclose(back, b)
    assert np.allclose(compose(a, inverse(a)), (0.0, 0.0, 0.0), atol=1e-12)


def test_raycast_hits_a_wall_at_the_right_distance():
    wall = np.array([[3.0, -1.0, 3.0, 1.0]])
    r = raycast((0.0, 0.0), np.array([0.0, math.pi]), wall)
    assert math.isclose(r[0], 3.0) and math.isinf(r[1])


def test_same_seed_same_trial_and_different_seed_differs():
    cfg = fs.nominal()
    a = fs.run_trial(cfg, "actuator_aware", 3, base_seed=1)
    b = fs.run_trial(cfg, "actuator_aware", 3, base_seed=1)
    c = fs.run_trial(cfg, "actuator_aware", 4, base_seed=1)
    assert a.as_dict() == b.as_dict()
    assert a.as_dict() != c.as_dict()


def test_both_controllers_face_the_same_scenario():
    cfg = fs.nominal()
    a, b = TrialRunner(cfg, "baseline", 7), TrialRunner(cfg, "actuator_aware", 7)
    assert a.pallet_pose == b.pallet_pose and a.vehicle.pose == b.vehicle.pose
    assert a.vehicle.steer_bias == b.vehicle.steer_bias and a.lidar.mount_true == b.lidar.mount_true


def test_ideal_simulator_always_loads_the_pallet():
    for ctrl in fs.CONTROLLERS:
        for k in range(5):
            assert fs.run_trial(fs.ideal(), ctrl, k).outcome == "success"


def _scan_of_pallet(view_deg: float, rng_m: float = 4.0):
    cfg = fs.ideal()
    pallet = (10.0, 15.0, math.pi / 2)
    edge = compose(pallet, (-0.5 * cfg.pallet.depth, 0.0, 0.0))
    segs = []
    for y0, y1 in cfg.pallet.blocks:
        segs += rect_segments(0.0, cfg.pallet.depth, y0, y1, edge)
    segs += rect_segments(0.0, cfg.room.width, 0.0, cfg.room.height, (0.0, 0.0, 0.0))
    v = math.radians(view_deg)
    sensor = compose(edge, (-rng_m * math.cos(v), rng_m * math.sin(v), -v))
    lidar = Lidar2D(cfg.lidar, cfg.perception, (0.0, 0.0, 0.0))
    ranges = lidar.scan(sensor, np.array(segs), np.random.default_rng(0))
    ok = np.isfinite(ranges)
    det = LidarPalletEstimator(cfg.pallet).detect(ranges[ok] * np.cos(lidar.angles[ok]), ranges[ok] * np.sin(lidar.angles[ok]))
    return det, (None if det is None else relative(pallet, compose(sensor, det.pallet)))


def test_detection_places_the_pallet_from_an_entry_edge():
    det, err = _scan_of_pallet(view_deg=10.0)
    assert det.source == "entry"
    assert math.hypot(err[0], err[1]) < 0.02 and abs(err[2]) < math.radians(1.0)


def test_detection_places_the_pallet_from_a_side_edge_without_turning_it():
    det, err = _scan_of_pallet(view_deg=88.0)          # looking almost square at a side (wrong) edge
    assert det.source == "side"
    turned = abs((err[2] + math.pi / 2) % math.pi - math.pi / 2)
    assert math.hypot(err[0], err[1]) < 0.05 and turned < math.radians(3.0)


def test_new_scan_replaces_cached_points_it_can_see():
    angles = np.radians(np.arange(-60.0, 60.1, 1.0))
    m = LocalMap(angles)
    m.update(np.full(angles.size, 2.0), (0.0, 0.0, 0.0), t=0.0)      # something 2 m away
    m.update(np.full(angles.size, 5.0), (0.0, 0.0, 0.0), t=1.0)      # it is gone: beams now reach 5 m
    assert np.allclose(np.hypot(m.xy[:, 0], m.xy[:, 1]), 5.0)
    m.update(np.full(angles.size, 5.0), (0.0, 0.0, math.pi), t=2.0)  # look the other way: old view is kept
    assert m.xy.shape[0] == 2 * angles.size


def test_smaller_approach_distance_tolerates_less_angle():
    tight, loose = acceptable_angle(0.5, 1.524, 0.0), acceptable_angle(2.5, 1.524, 0.0)
    assert 0.0 < tight < loose
    assert acceptable_angle(1.5, 1.524, 0.05) < acceptable_angle(1.5, 1.524, 0.0)
    assert acceptable_angle(0.5, 1.524, 0.3) == 0.0      # lateral offset alone is already too much


def test_collision_check_stops_a_truck_driven_into_the_pallet_side():
    cfg = fs.override(fs.ideal(), {"pallet.angle_deg": 0.0, "pallet.x": 10.0, "pallet.y": 12.0})
    cfg = fs.override(cfg, {"variation.pallet_pos": 0.0, "variation.pallet_angle_deg": 0.0,
                            "variation.start_lateral": 0.0, "variation.start_heading_deg": 0.0})
    run = TrialRunner(cfg, manual=True)                 # truck drives straight up at the pallet's long side
    while not run.step((0.5, 0.0)):
        pass
    assert run.outcome == "collision_side" and run.alert.startswith("COLLISION")


def test_defaults_are_valid_and_out_of_range_input_is_refused():
    assert fs.validate(fs.nominal()) == [] and fs.validate(fs.ideal()) == []
    for path, value in (("pallet.width", 0.6),          # forks cannot fit between the outer blocks
                        ("pallet.weight_kg", 2000.0),   # over the truck's rated load
                        ("pallet.x", 25.0),             # outside the room
                        ("task.approach_distance", 0.0)):
        cfg = fs.override(fs.nominal(), {path: value})
        assert any(path in msg for msg in fs.validate(cfg)), path
        with pytest.raises(ValueError):
            TrialRunner(cfg)


def test_limits_follow_from_the_truck():
    lim = fs.limits(fs.nominal())
    assert lim["pallet.width"][0] == pytest.approx(0.77)        # 550 mm forks + two 100 mm blocks + clearance
    assert lim["pallet.weight_kg"][1] == 1500.0
    wider_forks = fs.override(fs.nominal(), {"forklift.fork_outer_width": 0.68})
    assert fs.limits(wider_forks)["pallet.width"][0] == pytest.approx(0.90)
    assert fs.validate(wider_forks) != []                       # a EUR pallet no longer fits those forks


def test_web_page_loads_every_module_the_package_needs():
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    listed = set(re.findall(r'"(\w+\.py)"', re.search(r"const FILES = \[(.*?)\];", html, re.S).group(1)))
    needed = {p.name for p in (ROOT / "forksim").glob("*.py")} - {"experiment.py", "report.py", "ablation.py"}
    assert listed == needed


def test_a_much_wider_pallet_is_still_recognised_and_loaded():
    # pockets 0.83 m wide: the entry edge must not fall apart into three separate blocks
    cfg = fs.override(fs.ideal(), {"pallet.width": 2.0, "variation.pallet_angle_deg": 0.0})
    run = TrialRunner(cfg, "actuator_aware", 0)
    seen = set()
    while not run.step():
        seen.update(e.kind for e in run.estimator.last_edges)
    assert "entry" in seen and run.outcome == "success"
