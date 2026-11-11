"""Thin bridge between the simulator and the web page.

The page runs this same package in the browser (Pyodide) and talks to it
through four functions that take and return JSON strings.
"""
from __future__ import annotations

import json
import math
from collections import Counter
from dataclasses import replace

import numpy as np

from .behaviors import CONTROLLERS
from .config import Variation, nominal, override, scaled
from .geometry import compose, relative
from .trial import TrialRunner, run_trial

_run: TrialRunner | None = None
_scan_id = -1

# what the Properties tab may edit: label, config path, unit, step
PROPERTIES = [
    ["Room", [["width", "room.width", "m", 1], ["height", "room.height", "m", 1]]],
    ["Pallet", [["x", "pallet.x", "m", 0.1], ["y", "pallet.y", "m", 0.1], ["angle", "pallet.angle_deg", "deg", 5],
                ["size x (along forks)", "pallet.length", "m", 0.05], ["size y (entry edge)", "pallet.width", "m", 0.05],
                ["size z", "pallet.height", "m", 0.01], ["weight", "pallet.weight_kg", "kg", 50]]],
    ["Forklift", [["start x", "forklift.x", "m", 0.1], ["start y", "forklift.y", "m", 0.1],
                  ["start angle", "forklift.angle_deg", "deg", 5], ["max speed", "forklift.max_speed", "m/s", 0.1],
                  ["max steer angle", "forklift.max_steer_deg", "deg", 5],
                  ["min turning radius", "forklift.min_turn_radius", "m", 0.1]]],
    ["Task", [["approach distance D", "task.approach_distance", "m", 0.1],
              ["accepted share of possible error", "task.handover_margin", "", 0.1]]],
]


def _config(p: dict):
    cfg = scaled(float(p.get("perception_scale", 1.0)), float(p.get("control_scale", 1.0)))
    cfg = override(cfg, {k: float(v) for k, v in p.get("props", {}).items()})
    if not p.get("vary", True):
        cfg = replace(cfg, variation=Variation(0.0, 0.0, 0.0, 0.0))
    return cfg


def _r(x, nd=3):
    return None if x != x else round(float(x), nd)


def describe() -> str:
    """Defaults for the Properties tab."""
    cfg = nominal()
    groups = [[title, [[label, path, unit, step, getattr(getattr(cfg, path.split(".")[0]), path.split(".")[1])]
                       for label, path, unit, step in rows]] for title, rows in PROPERTIES]
    return json.dumps(dict(groups=groups, controllers=list(CONTROLLERS)))


def start(params_json: str) -> str:
    """Begin a trial. Returns the static picture: room, pallet (truth) and truck dimensions."""
    global _run, _scan_id
    p = json.loads(params_json)
    cfg = _config(p)
    manual = p.get("controller") == "manual"
    _run = TrialRunner(cfg, "actuator_aware" if manual else p["controller"], int(p.get("trial", 0)),
                       int(p.get("seed", 0)), manual=manual)
    _scan_id = -1
    f, pl = cfg.forklift, cfg.pallet
    return json.dumps(dict(
        room=[cfg.room.width, cfg.room.height],
        pallet=dict(pose=[_r(v, 4) for v in _run.pallet_pose], depth=pl.depth, width=pl.width,
                    blocks=[list(b) for b in pl.blocks], weight=pl.weight_kg),
        truck=dict(body_rear=f.body_rear, width=f.width, heel=f.fork_heel, reach=f.reach, fork_width=f.fork_width,
                   fork_spacing=f.fork_spacing, wheelbase=f.wheelbase, rated_load=f.rated_load_kg),
        approach_distance=cfg.task.approach_distance))


def _world(pose_true, odom, pt):
    """Something the robot holds in its odometry frame, drawn where the robot believes it is."""
    return compose(pose_true, relative(odom, pt))


def advance(n: int, action_json: str = "") -> str:
    """Run n ticks (20 ms each) and return what to draw."""
    global _scan_id
    run = _run
    action = tuple(json.loads(action_json)) if action_json else None
    for _ in range(int(n)):
        if run.step(action):
            break
    veh, ctrl = run.vehicle, run.ctrl
    lat, ang = run.true_params()
    s = dict(t=_r(run.t, 2), done=run.done, outcome=run.outcome, alert=run.alert,
             phase=ctrl.phase if not run.manual else "manual", step=ctrl.step_name if not run.manual else "manual drive",
             truck=[_r(veh.x, 4), _r(veh.y, 4), _r(veh.th, 4)], steer=_r(veh.steer + veh.steer_bias, 3), v=_r(veh.v, 3),
             lat=_r(lat, 1), ang=_r(ang, 2), lat_est=_r(ctrl.lat_est, 1), ang_est=_r(ctrl.angle_est, 2),
             depth=_r(run.depth, 3), det_err=[_r(run.det_err[0], 1), _r(run.det_err[1], 2)],
             angle_limit=_r(ctrl.angle_limit, 2))
    if ctrl.pallet is not None:
        s["pallet_est"] = [_r(v, 4) for v in _world(veh.pose, veh.odom, ctrl.pallet)]
        if ctrl._edge_end:
            s["edge_est"] = [_r(v, 4) for v in _world(veh.pose, veh.odom, ctrl.edge())]
            s["approach"] = [_r(v, 4) for v in _world(veh.pose, veh.odom, ctrl.approach_pose())]
    if run._k // run._lidar_every != _scan_id and run.ranges is not None:
        _scan_id = run._k // run._lidar_every
        # the latest scan, drawn where it really hit
        sx, sy, sth = compose(run.scan_pose, run.lidar.mount_true)
        ok = np.isfinite(run.ranges)
        a = run.lidar.angles[ok] + sth
        s["scan"] = np.round(np.column_stack((sx + run.ranges[ok] * np.cos(a), sy + run.ranges[ok] * np.sin(a))), 3).ravel().tolist()
        # cached map points (everything that is not from the latest scan), drawn where the robot believes they are
        old = run.map.xy[: max(0, run.map.xy.shape[0] - run.map.n_fresh)]
        if old.shape[0]:
            px, py = run.map.in_frame(veh.odom)
            px, py = px[: old.shape[0]], py[: old.shape[0]]
            c, sn = math.cos(veh.th), math.sin(veh.th)
            s["cached"] = np.round(np.column_stack((veh.x + c * px - sn * py, veh.y + sn * px + c * py)), 2).ravel().tolist()
        else:
            s["cached"] = []
        edges = []
        for e in getattr(run.estimator, "last_edges", []):
            if e.kind == "wall":
                continue
            ex, ey, eyaw = compose(veh.pose, e.pose)
            hx, hy = -math.sin(eyaw) * e.length / 2, math.cos(eyaw) * e.length / 2
            edges.append([e.kind, _r(ex - hx), _r(ey - hy), _r(ex + hx), _r(ey + hy)])
        s["edges"] = edges
    if run.done:
        s["result"] = {k: (_r(v, 2) if isinstance(v, float) else v) for k, v in run.result().as_dict().items()}
    return json.dumps(s)


def batch(params_json: str, first: int, count: int) -> str:
    """Run `count` trials starting at index `first` for every controller; returns outcomes."""
    p = json.loads(params_json)
    cfg = _config(p)
    out = {}
    for ctrl in CONTROLLERS:
        res = [run_trial(cfg, ctrl, k, int(p.get("seed", 0))) for k in range(first, first + count)]
        out[ctrl] = dict(n=len(res), success=sum(r.success for r in res),
                         outcomes=dict(Counter(r.outcome for r in res)))
    return json.dumps(out)
