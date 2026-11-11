"""Monte Carlo experiments.

    python -m forksim.experiment                 # everything, ~15k trials
    python -m forksim.experiment --quick         # 10x fewer trials, for a fast look
    python -m forksim.experiment --seed 7        # a different, equally reproducible set

Writes results/trials.csv (one row per trial), results/perception.csv and
results/summary.json, then calls forksim.report to draw the figures.

Every trial is identified by (seed, trial index); the same index means the same
pallet pose, start pose and per-trial biases for both controllers and for every
error level, so all comparisons are paired.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from collections import Counter
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from .behaviors import CONTROLLERS
from .config import SimConfig, nominal, override, scaled
from .geometry import compose, rect_segments, relative, wrap_half
from .perception import LidarPalletEstimator
from .sensors import Lidar2D
from .trial import OUTCOMES, run_trial

SCALES = (0.0, 0.5, 1.0, 1.5, 2.0)
STRESS = 1.5   # sensitivity is measured around STRESS x nominal, where pick-ups start to fail

# one-at-a-time sensitivity: label -> config field. Each is set to 0x and 2x its stressed value.
SENSITIVITY = {
    "lidar range noise": "perception.lidar_range_sigma",
    "lidar dropout": "perception.lidar_dropout",
    "lidar mount yaw error": "perception.lidar_yaw_bias_sigma_deg",
    "lidar mount lateral error": "perception.lidar_lateral_bias_sigma",
    "scan/pose time offset": "perception.time_sync_error",
    "steering zero offset": "control.steer_bias_sigma_deg",
    "steering lag": "control.steer_lag",
    "steering rate limit": "control.steer_rate_limit_deg",
    "command latency": "control.cmd_latency",
    "drive lag": "control.speed_lag",
    "wheel scale error": "control.speed_scale_sigma",
    "floor slip": "control.slip_sigma_deg",
}


def _level(cfg: SimConfig, path: str, factor: float) -> SimConfig:
    group, name = path.split(".")
    value = getattr(getattr(cfg, group), name)
    if name == "steer_rate_limit_deg":       # a limit: "no error" is infinite, "twice as bad" is half
        return override(cfg, {path: math.inf if factor == 0 else value / factor})
    return override(cfg, {path: value * factor})


def build_cells(n_scale: float) -> list:
    """Each cell is one (config, controller) combination run for n trials."""
    n = lambda base: max(20, int(round(base * n_scale)))
    stressed = scaled(STRESS, STRESS)
    cells = []
    for ctrl in CONTROLLERS:
        for label, cfg in (("ideal", scaled(0, 0)), ("nominal", nominal()), ("stressed", stressed)):
            cells.append(dict(exp="headline", label=label, controller=ctrl, cfg=cfg, n=n(500)))
        for ps in SCALES:
            for cs in SCALES:
                cells.append(dict(exp="grid", label=f"p{ps:g}_c{cs:g}", perception_scale=ps, control_scale=cs,
                                  controller=ctrl, cfg=scaled(ps, cs), n=n(120)))
        for label, path in SENSITIVITY.items():
            for factor in (0.0, 2.0):
                cells.append(dict(exp="sensitivity", label=label, factor=factor, controller=ctrl,
                                  cfg=_level(stressed, path, factor), n=n(150)))
    return cells


def _job(args):
    cell_idx, cfg, controller, trial, seed = args
    return cell_idx, run_trial(cfg, controller, trial, seed).as_dict()


def wilson(k: int, n: int, z: float = 1.96):
    """95 % confidence interval for a success rate."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (max(0.0, centre - half), min(1.0, centre + half))


def _pct(values, q):
    v = sorted(abs(x) for x in values if x == x)
    return v[min(len(v) - 1, int(len(v) * q))] if v else None


def summarise(cell: dict, rows: list) -> dict:
    k, n = sum(r["success"] for r in rows), len(rows)
    lo, hi = wilson(k, n)
    out = {key: val for key, val in cell.items() if key != "cfg"}
    out.update(n=n, successes=k, success_rate=k / n, ci_low=lo, ci_high=hi,
               outcomes=dict(Counter(r["outcome"] for r in rows)),
               bad_detections=sum(r["bad_detections"] for r in rows),
               mean_duration_s=sum(r["duration_s"] for r in rows) / n)
    for key in ("det_pos_mm", "det_angle_deg", "handover_lat_mm", "handover_angle_deg", "est_lat_mm",
                "est_angle_deg", "entry_lat_mm", "entry_angle_deg", "load_lat_mm", "load_angle_deg"):
        out[key + "_p50"] = _pct([r[key] for r in rows], 0.5)
        out[key + "_p90"] = _pct([r[key] for r in rows], 0.9)
    return out


# ------------------------------------------------------------------ perception on its own
def perception_study(n: int, seed: int) -> list:
    """Single scans of a pallet from known poses: how well does detection place it?
    No truck motion, no behaviour - this isolates the lidar pipeline."""
    rows = []
    for scale in (1.0, 2.0):
        cfg = scaled(scale, 0.0)
        est = LidarPalletEstimator(cfg.pallet)
        pallet = (10.0, 15.0, math.pi / 2)
        edge = compose(pallet, (-0.5 * cfg.pallet.depth, 0.0, 0.0))
        segs = []
        for y0, y1 in cfg.pallet.blocks:
            segs += rect_segments(0.0, cfg.pallet.depth, y0, y1, edge)
        segs += rect_segments(0.0, cfg.room.width, 0.0, cfg.room.height, (0.0, 0.0, 0.0))
        segs = np.array(segs)
        for rng_m in (2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0):
            rng = np.random.default_rng([seed, int(scale * 10), int(rng_m * 10)])
            lidar = Lidar2D(cfg.lidar, cfg.perception, (0.0, 0.0, 0.0))
            for _ in range(n):
                view = math.radians(rng.uniform(-80.0, 80.0))     # how far round the pallet we stand
                skew = math.radians(rng.uniform(-10.0, 10.0))     # sensor not pointing straight at it
                sensor = compose(edge, (-rng_m * math.cos(view), rng_m * math.sin(view), -view + skew))
                ranges = lidar.scan(sensor, segs, rng)
                ok = np.isfinite(ranges)
                det = est.detect(ranges[ok] * np.cos(lidar.angles[ok]), ranges[ok] * np.sin(lidar.angles[ok]))
                row = dict(noise_scale=scale, range_m=rng_m, view_deg=math.degrees(view), detected=det is not None,
                           source=det.source if det else "", turned_90=False,
                           pos_mm=float("nan"), lat_mm=float("nan"), angle_deg=float("nan"))
                if det is not None:
                    err = relative(pallet, compose(sensor, det.pallet))
                    row["turned_90"] = abs(wrap_half(err[2])) > math.radians(45.0)
                    if not row["turned_90"]:
                        row.update(pos_mm=math.hypot(err[0], err[1]) * 1e3, lat_mm=err[1] * 1e3,
                                   angle_deg=math.degrees(wrap_half(err[2])))
                rows.append(row)
    return rows


# ------------------------------------------------------------------ main
def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="results")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true", help="10x fewer trials")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cells = build_cells(0.1 if args.quick else 1.0)
    jobs = [(i, c["cfg"], c["controller"], t, args.seed) for i, c in enumerate(cells) for t in range(c["n"])]
    print(f"{len(cells)} cells, {len(jobs)} trials, {args.workers} workers, seed {args.seed}", flush=True)

    t0 = time.perf_counter()
    rows = [[] for _ in cells]
    with Pool(args.workers) as pool:
        for done, (i, res) in enumerate(pool.imap_unordered(_job, jobs, chunksize=16), 1):
            rows[i].append(res)
            if done % 1000 == 0:
                print(f"  {done}/{len(jobs)}  {time.perf_counter() - t0:.0f} s", flush=True)
    for r in rows:
        r.sort(key=lambda d: d["trial"])

    meta_keys = ("exp", "label", "perception_scale", "control_scale", "factor")
    with open(out / "trials.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[*meta_keys, *rows[0][0].keys()])
        w.writeheader()
        for cell, cell_rows in zip(cells, rows):
            meta = {k: cell.get(k, "") for k in meta_keys}
            w.writerows({**meta, **r} for r in cell_rows)

    perc = perception_study(60 if args.quick else 300, args.seed)
    with open(out / "perception.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(perc[0].keys()))
        w.writeheader()
        w.writerows(perc)

    summary = dict(seed=args.seed, trials=len(jobs), outcomes=list(OUTCOMES), scales=list(SCALES), stress=STRESS,
                   cells=[summarise(c, r) for c, r in zip(cells, rows)])
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(f"done in {time.perf_counter() - t0:.0f} s -> {out}/", flush=True)

    try:
        from . import report
        report.main(["--results", str(out)])
    except Exception as exc:        # the data is already on disk; figures can be redrawn
        print(f"report failed ({exc!r}); rerun with: python -m forksim.report --results {out}")


if __name__ == "__main__":
    main()
