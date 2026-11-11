"""Turn results/ into figures and a results table.

    python -m forksim.report [--results results]
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .config import nominal

NICE = {"baseline": "Baseline (tuned in ideal sim)", "actuator_aware": "Actuator-aware"}
SIM = {"ideal": "ideal sim", "nominal": "assumed real truck (1x)", "stressed": "stressed"}
COLORS = {"success": "#2a9d8f", "collision_fork_face": "#e76f51", "collision_fork_wall": "#f4a261",
          "collision_side": "#9b2226", "collision_body": "#bc4749", "no_pallet_detected": "#6d597a",
          "no_approach_position": "#b5838d", "approach_error_too_large": "#e9c46a", "lost_pallet": "#8d99ae",
          "left_room": "#adb5bd", "timeout": "#264653"}


def tolerance_mm() -> float:
    """Lateral room a fork has inside its pocket, plus the tolerated overlap."""
    cfg = nominal()
    b, f = cfg.pallet.blocks, cfg.forklift
    outer = b[2][0] - (0.5 * f.fork_spacing + 0.5 * f.fork_width)
    inner = (0.5 * f.fork_spacing - 0.5 * f.fork_width) - b[1][1]
    return (min(outer, inner) + cfg.task.contact_tol) * 1e3


def _num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def _p90(values):
    v = sorted(abs(x) for x in values if x == x)
    return v[int(0.9 * (len(v) - 1))] if v else float("nan")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default="results")
    args = ap.parse_args(argv)
    out = Path(args.results)
    summary = json.loads((out / "summary.json").read_text())
    cells = summary["cells"]
    with open(out / "trials.csv", newline="") as f:
        trials = list(csv.DictReader(f))
    with open(out / "perception.csv", newline="") as f:
        perc = list(csv.DictReader(f))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    ctrls = list(NICE)
    scales, stress = summary["scales"], summary["stress"]
    SIM["stressed"] = f"stressed ({stress:g}x)"
    cell = lambda exp, label, ctrl, **kw: next(
        c for c in cells if c["exp"] == exp and c["label"] == label and c["controller"] == ctrl
        and all(c.get(k) == v for k, v in kw.items()))

    # ---- 1. success over perception error x control error
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True)
    for ax, ctrl in zip(axes, ctrls):
        grid = np.full((len(scales), len(scales)), np.nan)
        for c in cells:
            if c["exp"] == "grid" and c["controller"] == ctrl:
                grid[scales.index(c["control_scale"]), scales.index(c["perception_scale"])] = 100 * c["success_rate"]
        im = ax.imshow(grid, origin="lower", cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")
        for i in range(len(scales)):
            for j in range(len(scales)):
                ax.text(j, i, f"{grid[i, j]:.0f}", ha="center", va="center", fontsize=10)
        ax.set_xticks(range(len(scales)), [f"{s:g}x" for s in scales])
        ax.set_yticks(range(len(scales)), [f"{s:g}x" for s in scales])
        ax.set_xlabel("perception error (x nominal)")
        ax.set_title(NICE[ctrl])
    axes[0].set_ylabel("control error (x nominal)")
    fig.colorbar(im, ax=axes, label="pallets loaded %", shrink=0.9)
    fig.suptitle("Pallets loaded as the simulator is made less ideal  (0x, 0x = ideal sim; 1x, 1x = assumed real truck)")
    fig.savefig(out / "fig1_success_grid.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 2. one-at-a-time sensitivity around the stressed point
    labels = []
    for c in cells:
        if c["exp"] == "sensitivity" and c["label"] not in labels:
            labels.append(c["label"])
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), sharey=True)
    for ax, ctrl in zip(axes, ctrls):
        ref = 100 * cell("headline", "stressed", ctrl)["success_rate"]
        y = np.arange(len(labels))
        for factor, color, name in ((0.0, "#2a9d8f", "this error removed"), (2.0, "#e76f51", "this error doubled")):
            vals = [100 * cell("sensitivity", lab, ctrl, factor=factor)["success_rate"] for lab in labels]
            ax.barh(y + (0.2 if factor else -0.2), np.array(vals) - ref, height=0.38, left=ref, color=color, label=name)
        ax.axvline(ref, color="k", lw=1)
        ax.set_yticks(y, labels)
        ax.invert_yaxis()
        ax.set_xlim(0, 100)
        ax.set_xlabel(f"pallets loaded %  (black line = all errors at {stress:g}x)")
        ax.set_title(NICE[ctrl])
    axes[1].legend(loc="upper left", fontsize=9)
    fig.suptitle(f"Which single error source matters? One changed at a time, the rest held at {stress:g}x nominal")
    fig.savefig(out / "fig2_sensitivity.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 3. the two alignment errors when the forks reach the pallet
    tol = tolerance_mm()
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True, sharex=True)
    for ax, ctrl in zip(axes, ctrls):
        rows = [t for t in trials if t["exp"] == "headline" and t["label"] == "stressed" and t["controller"] == ctrl]
        for ok, color, name in ((True, "#2a9d8f", "loaded"), (False, "#e76f51", "collision")):
            sel = [t for t in rows if (t["success"] == "True") == ok and _num(t["entry_lat_mm"]) == _num(t["entry_lat_mm"])]
            ax.scatter([_num(t["entry_angle_deg"]) for t in sel], [_num(t["entry_lat_mm"]) for t in sel],
                       s=8, color=color, alpha=0.6, label=f"{name} ({len(sel)})")
        # the room a fork has in its pocket. The angle is not a hard limit here: the truck keeps
        # steering while it drives in, and it pivots near its fork tips, so it can still straighten up.
        ax.axhline(tol, color="k", ls="--", lw=1, label=f"fork clearance +/-{tol:.0f} mm")
        ax.axhline(-tol, color="k", ls="--", lw=1)
        never = sum(1 for t in rows if _num(t["entry_lat_mm"]) != _num(t["entry_lat_mm"]))
        ax.set_title(f"{NICE[ctrl]}\n({never} of {len(rows)} trials never got the forks to the edge)", fontsize=10)
        ax.set_xlabel("angle: pallet edge vs carriage face (deg)")
        ax.set_xlim(-5, 5)
        ax.set_ylim(-90, 90)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    axes[0].set_ylabel("lateral offset at entry edge (mm)")
    fig.suptitle(f"The two alignment errors, ground truth, as the fork tips cross the entry edge (errors at {stress:g}x)", y=1.04)
    fig.savefig(out / "fig3_alignment.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 4. how trials end
    fig, ax = plt.subplots(figsize=(9, 4.2))
    bars = [(lab, ctrl) for lab in ("ideal", "nominal", "stressed") for ctrl in ctrls]
    left = np.zeros(len(bars))
    for outcome in summary["outcomes"]:
        vals = [100 * cell("headline", lab, ctrl)["outcomes"].get(outcome, 0) / cell("headline", lab, ctrl)["n"]
                for lab, ctrl in bars]
        if any(vals):
            ax.barh(range(len(bars)), vals, left=left, color=COLORS[outcome], label=outcome.replace("_", " "))
            left += np.array(vals)
    ax.set_yticks(range(len(bars)), [f"{SIM[lab]} / {NICE[ctrl].split(' (')[0]}" for lab, ctrl in bars])
    ax.invert_yaxis()
    ax.set_xlabel("% of trials")
    ax.legend(fontsize=8, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    fig.savefig(out / "fig4_outcomes.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 5. perception alone: how well a single scan places the pallet, and how that changes with range
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    perc_table = []
    square = lambda q: abs(_num(q["view_deg"])) < 35.0        # looking at an entry edge more or less head on
    for scale, color in ((1.0, "#264653"), (2.0, "#e76f51")):
        ranges = sorted({_num(q["range_m"]) for q in perc})
        det, turned, lat90, ang90, lat_all = [], [], [], [], []
        for r in ranges:
            rows = [q for q in perc if _num(q["noise_scale"]) == scale and _num(q["range_m"]) == r]
            good = [q for q in rows if q["detected"] == "True" and q["turned_90"] == "False"]
            head_on = [q for q in good if square(q)]
            det.append(100 * len(good) / len(rows))
            turned.append(100 * sum(q["turned_90"] == "True" for q in rows) / len(rows))
            lat90.append(_p90([_num(q["lat_mm"]) for q in head_on]))
            ang90.append(_p90([_num(q["angle_deg"]) for q in head_on]))
            lat_all.append(_p90([_num(q["lat_mm"]) for q in good]))
            perc_table.append((scale, r, det[-1], turned[-1], lat90[-1], ang90[-1], lat_all[-1]))
        axes[0].plot(ranges, det, "o-", color=color, label=f"{scale:g}x noise")
        axes[0].plot(ranges, turned, "x--", color=color, alpha=0.6)
        axes[1].plot(ranges, lat90, "o-", color=color, label=f"{scale:g}x, head on")
        axes[1].plot(ranges, lat_all, ":", color=color, label=f"{scale:g}x, any view")
        axes[2].plot(ranges, ang90, "o-", color=color)
    axes[0].set_title("pallet found % (solid)\nfound but turned 90 deg % (dashed)", fontsize=10)
    axes[1].set_title("pallet centre, lateral error, 90th pct (mm)", fontsize=10)
    axes[2].set_title("pallet angle error, head on, 90th pct (deg)", fontsize=10)
    axes[1].axhline(tol, color="k", ls="--", lw=1)
    axes[1].set_ylim(0, 120)
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.set_xlabel("range to pallet (m)")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=9)
    fig.suptitle("Detection on its own, single scans: a closer look is a better look", y=1.04)
    fig.savefig(out / "fig5_perception.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 6. approach distance D
    d_values = summary["d_values"]
    styles = {("baseline", 1.0): ("#e76f51", "o-"), ("baseline", stress): ("#e76f51", "s--"),
              ("actuator_aware", 1.0): ("#264653", "o-"), ("actuator_aware", stress): ("#264653", "s--")}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    d_table = []
    for (ctrl, level), (color, style) in styles.items():
        cs = [cell("d_sweep", f"D={d:g}", ctrl, level=level) for d in d_values]
        name = f"{NICE[ctrl].split(' (')[0]}, {level:g}x errors"
        axes[0].plot(d_values, [100 * c["success_rate"] for c in cs], style, color=color, label=name)
        axes[1].plot(d_values, [100 * c["outcomes"].get("approach_error_too_large", 0) / c["n"] for c in cs],
                     style, color=color, label=name)
        d_table.append((ctrl, level, cs))
    axes[0].set_ylabel("pallets loaded %")
    axes[0].set_ylim(0, 102)
    axes[0].set_title("Success", fontsize=10)
    axes[1].set_ylabel("% of trials")
    axes[1].set_title("Rejected at the approach target: arrival angle beyond what D allows", fontsize=10)
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.set_xlabel("approach distance D (m)")
        ax.grid(alpha=0.3)
    fig.suptitle("Approach distance D: too short leaves no room to straighten up, too long starts aligning from too far", y=1.02)
    fig.savefig(out / "fig6_approach_distance.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 7. what the truck is facing when it starts
    layouts = summary["layouts"]
    levels = (0.0, 1.0, stress)
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), sharey=True)
    for ax, level in zip(axes, levels):
        x = np.arange(len(layouts))
        for k, ctrl in enumerate(ctrls):
            vals = [100 * cell("layout", lab, ctrl, level=level)["success_rate"] for lab in layouts]
            ax.bar(x + (k - 0.5) * 0.38, vals, width=0.36, color=("#e76f51", "#264653")[k], label=NICE[ctrl].split(" (")[0])
            for xi, v in zip(x, vals):
                ax.text(xi + (k - 0.5) * 0.38, v + 1.5, f"{v:.0f}", ha="center", fontsize=8)
        ax.set_xticks(x, [lab.replace(" ahead", "\nahead") for lab in layouts], fontsize=9)
        ax.set_ylim(0, 125)
        ax.set_title("ideal sim" if level == 0 else f"errors at {level:g}x", fontsize=10)
        ax.grid(alpha=0.3, axis="y")
    axes[0].set_ylabel("pallets loaded %")
    axes[2].legend(fontsize=8, loc="upper right")
    fig.suptitle("Where the truck starts: facing an entry edge, a corner, or a side (wrong) edge", y=1.03)
    fig.savefig(out / "fig7_start_layout.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- 8. lag between perception and control
    lag_cells = {ctrl: [cell("lag", f"lag {f:g}x", ctrl) for f in summary["lag_factors"]] for ctrl in ctrls}
    fig, ax = plt.subplots(figsize=(6.5, 4))
    for ctrl, color in zip(ctrls, ("#e76f51", "#264653")):
        ax.plot([c["lag_ms"] for c in lag_cells[ctrl]], [100 * c["success_rate"] for c in lag_cells[ctrl]], "o-",
                color=color, label=NICE[ctrl].split(" (")[0])
    ax.set_xlabel("lag between perception and control (ms): stale scan + command delay")
    ax.set_ylabel("pallets loaded %")
    ax.set_ylim(0, 102)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title("Lag swept alone, every other error at 1x", fontsize=10)
    fig.savefig(out / "fig8_lag.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---- results table
    f = lambda v, d: "-" if v is None else f"{v:.{d}f}"

    def row(c):
        o = ", ".join(f"{k.replace('_', ' ')} {v}" for k, v in sorted(c["outcomes"].items(), key=lambda kv: -kv[1])
                      if k != "success")
        return (f"| {SIM[c['label']]} | {NICE[c['controller']].split(' (')[0]} | {c['successes']}/{c['n']} | "
                f"{100 * c['success_rate']:.1f} % ({100 * c['ci_low']:.1f}-{100 * c['ci_high']:.1f}) | "
                f"{f(c['handover_lat_mm_p90'], 1)} / {f(c['handover_angle_deg_p90'], 2)} | "
                f"{f(c['entry_lat_mm_p90'], 1)} / {f(c['entry_angle_deg_p90'], 2)} | {f(c['mean_duration_s'], 0)} | {o or '-'} |")

    md = [f"# Results (seed {summary['seed']}, {summary['trials']} trials)", "",
          f"Lateral tolerance from fork/pocket geometry: +/-{tol:.1f} mm. "
          "Error columns are 90th percentiles of ground truth: lateral mm / angle deg.", "",
          "## Ideal simulator vs less ideal ones", "",
          "| Simulator | Controller | Loaded | Rate (95 % CI) | Error at approach target | Error at fork entry | Mean time (s) | Failures |",
          "|---|---|---|---|---|---|---|---|"]
    md += [row(c) for c in cells if c["exp"] == "headline"]
    md += ["", f"## One error source at a time (rest at {stress:g}x nominal)", "",
           "| Error source | Level | Baseline | Actuator-aware |", "|---|---|---|---|",
           "| (none changed) | - | " + " | ".join(
               f"{100 * cell('headline', 'stressed', c)['success_rate']:.1f} %" for c in ctrls) + " |"]
    for lab in labels:
        for factor, name in ((0.0, "removed"), (2.0, "doubled")):
            md.append(f"| {lab} | {name} | " + " | ".join(
                f"{100 * cell('sensitivity', lab, c, factor=factor)['success_rate']:.1f} %" for c in ctrls) + " |")
    md += ["", "## Approach distance D", "",
           "| D (m) | " + " | ".join(f"{NICE[c].split(' (')[0]} {lv:g}x" for c, lv, _ in d_table) + " |",
           "|---|" + "---|" * len(d_table)]
    for i, d in enumerate(d_values):
        md.append(f"| {d:g} | " + " | ".join(
            f"{100 * cs[i]['success_rate']:.1f} % (rejected {cs[i]['outcomes'].get('approach_error_too_large', 0)}/{cs[i]['n']})"
            for _, _, cs in d_table) + " |")
    md += ["", "## Lag between perception and control (rest at 1x)", "",
           "| Lag (ms) | Baseline | Actuator-aware | Baseline failures |", "|---|---|---|---|"]
    for i in range(len(summary["lag_factors"])):
        cb, ca = lag_cells["baseline"][i], lag_cells["actuator_aware"][i]
        fails = ", ".join(f"{k.replace('_', ' ')} {v}" for k, v in sorted(cb["outcomes"].items(), key=lambda kv: -kv[1]) if k != "success")
        md.append(f"| {cb['lag_ms']} | {cb['successes']}/{cb['n']} = {100 * cb['success_rate']:.0f} % | "
                  f"{ca['successes']}/{ca['n']} = {100 * ca['success_rate']:.0f} % | {fails or '-'} |")
    md += ["", "## Start layout", "", "| Facing at start | Errors | Baseline | Actuator-aware | Mean time (s) |", "|---|---|---|---|---|"]
    for lab in layouts:
        for level in levels:
            cs = [cell("layout", lab, c, level=level) for c in ctrls]
            md.append(f"| {lab} | {level:g}x | " + " | ".join(f"{c['successes']}/{c['n']} = {100 * c['success_rate']:.1f} %" for c in cs)
                      + f" | {cs[0]['mean_duration_s']:.0f} / {cs[1]['mean_duration_s']:.0f} |")
    md += ["", "## Detection alone (single scans)", "",
           "| Noise | Range (m) | Pallet found | Found but turned 90 deg | lateral p90 head on (mm) | angle p90 head on (deg) | lateral p90 any view (mm) |",
           "|---|---|---|---|---|---|---|"]
    md += [f"| {s:g}x | {r:g} | {d:.1f} % | {w:.1f} % | {l:.1f} | {a:.2f} | {la:.1f} |" for s, r, d, w, l, a, la in perc_table]
    (out / "summary.md").write_text("\n".join(md) + "\n")
    print((out / "summary.md").read_text())


if __name__ == "__main__":
    main()
