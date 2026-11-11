"""Which half of the actuator-aware controller does the work?

    python -m forksim.ablation

The improvement is two mechanisms. This runs each one alone, both, and neither,
on the same trials, under the conditions where the two controllers differ.
About one minute.
"""
from __future__ import annotations

import math
import os
from collections import Counter
from multiprocessing import Pool

from .behaviors import BehaviorParams
from .config import nominal, override, scaled
from .trial import run_trial

VARIANTS = {
    "neither (baseline)": BehaviorParams(),
    "delay prediction only": BehaviorParams(delay_comp=0.25),
    "wheel waiting only": BehaviorParams(steer_wait_deg=8.0),
    "both (actuator-aware)": BehaviorParams(delay_comp=0.25, steer_wait_deg=8.0),
}
CONDITIONS = {
    "all errors 1.5x": (scaled(1.5, 1.5), 40),
    "lag 480 ms, rest 1x": (override(nominal(), {"perception.time_sync_error": 0.18, "control.cmd_latency": 0.30}), 20),
    "lag 640 ms, rest 1x": (override(nominal(), {"perception.time_sync_error": 0.24, "control.cmd_latency": 0.40}), 20),
    "side edge ahead, 1x": (override(nominal(), {"pallet.angle_deg": 0.0}), 20),
    "control 2x, perception ideal": (scaled(0.0, 2.0), 20),
}


def _job(args):
    cond, var, k = args
    r = run_trial(CONDITIONS[cond][0], "baseline", k, ctrl_params=VARIANTS[var])
    return cond, var, r.success, r.outcome, r.duration_s, r.entry_angle_deg


def main() -> None:
    jobs = [(c, v, k) for c, (_, n) in CONDITIONS.items() for v in VARIANTS for k in range(n)]
    with Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
        out = pool.map(_job, jobs, chunksize=4)
    for cond, (_, n) in CONDITIONS.items():
        print(f"\n{cond}  (n={n})")
        for var in VARIANTS:
            rows = [o for o in out if o[0] == cond and o[1] == var]
            ok = sum(o[2] for o in rows)
            ang = sorted(abs(o[5]) for o in rows if o[5] == o[5])
            a90 = ang[int(0.9 * (len(ang) - 1))] if ang else math.nan
            fails = dict(Counter(o[3] for o in rows if not o[2]))
            print(f"  {var:24s} {ok:2d}/{n} = {100 * ok / n:3.0f}%  mean time {sum(o[4] for o in rows) / n:4.0f}s"
                  f"  entry angle p90 {a90:5.2f} deg  {fails or ''}")


if __name__ == "__main__":
    main()
