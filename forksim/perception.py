"""Perception: lidar points in, pallet pose out. No prior knowledge of the world.

`detect()` returns the pallet's position and angle relative to the robot. It is
a fit to noisy lidar points, so it is noisy too.

A pallet has four edges: two ENTRY edges (short, three blocks with two fork
pockets between them) and two SIDE edges (long, solid). Forks only go in
through an entry edge, so the detector has to tell the two kinds apart. Either
kind is enough to place the pallet; an entry edge places it more precisely.

The only thing known in advance is what a pallet of this type looks like (edge
lengths and block pattern). Where the pallet is, and what else is in the room,
comes from the points alone:

    1. pull straight lines out of the points, strongest first
    2. long lines are walls or racking: discard
    3. short lines are edge candidates: classify by length and block pattern
    4. pallet centre = half a pallet behind the edge, along the edge normal

Points are given in the robot frame. `OraclePerception` has the same signature
and returns truth plus noise, so control work can skip this pipeline.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import Pallet
from .geometry import wrap, wrap_half


@dataclass
class Edge:
    kind: str          # "entry", "side", "wall" or "other"
    pose: tuple        # edge centre, x axis pointing away from the sensor, robot frame
    length: float      # measured extent along the edge
    block_frac: float  # share of points that fall where an entry edge has blocks
    n_points: int


@dataclass
class Detection:
    pallet: tuple      # pallet centre (x, y) and axis angle, robot frame. The axis is the direction
                       # forks travel through the pallet; it is only defined modulo 180 deg.
    source: str        # "entry" or "side": the kind of edge it was derived from
    edge: Edge


class LidarPalletEstimator:
    def __init__(self, pallet: Pallet, sensor_xy=(0.0, 0.0), inlier_tol: float = 0.04,
                 min_points: int = 6, n_hypotheses: int = 80, max_lines: int = 10):
        self.pallet, self.sensor_xy = pallet, sensor_xy
        self.tol, self.min_points, self.max_lines = inlier_tol, min_points, max_lines
        # fixed sampling pattern: the detector is a deterministic function of its input
        self._u = np.random.default_rng(2024).random((max_lines, n_hypotheses, 2))
        self._block_c = np.array([0.5 * (a + b) for a, b in pallet.blocks])
        self._block_h = np.array([0.5 * (b - a) for a, b in pallet.blocks])
        self._wall_len = max(pallet.width, pallet.depth) + 1.0   # anything straight and longer is not a pallet
        # Sizes that follow from this pallet type, so a wider or longer pallet is still recognised:
        pocket = max(b[0] - a[1] for a, b in zip(pallet.blocks, pallet.blocks[1:]))
        self._max_gap = max(0.35, pocket + 0.12)     # widest break allowed inside one edge = a fork pocket
        self._gate = max(1.6, 0.5 * math.hypot(pallet.width, pallet.depth) + 0.85)   # tracking window radius
        self.last_edges = []     # kept for display
        self.blob = None         # nearest pallet-sized thing (x, y), robot frame, even if no entry edge is visible

    # ------------------------------------------------------------ line fitting
    @staticmethod
    def _fit(px, py):
        mx, my = px.mean(), py.mean()
        dx, dy = px - mx, py - my
        ang = 0.5 * math.atan2(2.0 * float((dx * dy).sum()), float((dx * dx).sum() - (dy * dy).sum()))
        return mx, my, math.cos(ang), math.sin(ang)

    def _ransac_line(self, px, py, u):
        """Most populated straight line among the points: (inlier mask, mx, my, tx, ty) or None."""
        n = px.size
        if n < self.min_points:
            return None
        i, j = (u[:, 0] * n).astype(int), (u[:, 1] * n).astype(int)
        dx, dy = px[j] - px[i], py[j] - py[i]
        norm = np.hypot(dx, dy)
        valid = norm > 0.15
        norm = np.where(valid, norm, 1.0)
        nx, ny = -dy / norm, dx / norm
        dist = np.abs((px[None, :] - px[i][:, None]) * nx[:, None] + (py[None, :] - py[i][:, None]) * ny[:, None])
        counts = np.where(valid, (dist < self.tol).sum(axis=1), 0)
        mask = dist[int(counts.argmax())] < self.tol
        for _ in range(2):   # least-squares polish
            if mask.sum() < self.min_points:
                return None
            mx, my, tx, ty = self._fit(px[mask], py[mask])
            mask = np.abs((px - mx) * -ty + (py - my) * tx) < self.tol
        if mask.sum() < self.min_points:
            return None
        return (mask, *self._fit(px[mask], py[mask]))

    # ------------------------------------------------------------ edge finding
    def detect_edges(self, px: np.ndarray, py: np.ndarray) -> list:
        """Every straight edge in the points, classified. Points are in the robot frame."""
        edges = []
        for k in range(self.max_lines):
            line = self._ransac_line(px, py, self._u[k])
            if line is None:
                break
            mask, mx, my, tx, ty = line
            nx, ny = -ty, tx                    # edge normal, pointing away from the sensor = into the pallet
            if nx * (mx - self.sensor_xy[0]) + ny * (my - self.sensor_xy[1]) < 0:
                nx, ny = -nx, -ny
            tx, ty = -ny, nx
            idx = np.flatnonzero(mask)
            s = (px[idx] - mx) * tx + (py[idx] - my) * ty
            order = np.argsort(s)
            idx, s = idx[order], s[order]
            # longest unbroken run of points along the line (a pocket is the widest gap allowed)
            cuts = np.flatnonzero(np.diff(s) > self._max_gap)
            starts, ends = np.r_[0, cuts + 1], np.r_[cuts + 1, s.size]
            r = int((ends - starts).argmax())
            run = slice(starts[r], ends[r])
            if s[-1] - s[0] > self._wall_len and s.size - (ends[r] - starts[r]) >= self.min_points or \
                    s[run][-1] - s[run][0] > self._wall_len:
                # a wall: drop everything along it, with a generous band so its noise tail goes too
                edges.append(Edge("wall", (mx, my, math.atan2(ny, nx)), float(s[-1] - s[0]), 0.0, int(s.size)))
                keep = np.abs((px - mx) * nx + (py - my) * ny) > 0.15
                px, py = px[keep], py[keep]
                continue
            idx, s = idx[run], s[run]
            px_rest, py_rest = np.delete(px, idx), np.delete(py, idx)
            if s.size < self.min_points:
                px, py = px_rest, py_rest
                continue
            length = float(s[-1] - s[0])
            mid = 0.5 * float(s[-1] + s[0])
            # compare against the entry-edge pattern: block / pocket / block / pocket / block
            off = (s - mid)[:, None] - self._block_c[None, :]
            nearest = np.abs(off).argmin(axis=1)
            off = off[np.arange(s.size), nearest]
            on_block = np.abs(off) <= self._block_h[nearest] + 0.03
            block_frac = float(on_block.mean())
            if abs(length - self.pallet.width) <= 0.12 and block_frac >= 0.8:
                kind = "entry"
                mid += float(off[on_block].mean())   # centre from all block points, not just the two end points
            elif abs(length - self.pallet.depth) <= 0.18 and block_frac < 0.8:
                kind = "side"
            else:
                kind = "other"
            edges.append(Edge(kind, (mx + tx * mid, my + ty * mid, math.atan2(ny, nx)), length, block_frac, int(s.size)))
            px, py = px_rest, py_rest
        self.last_edges = edges
        return edges

    def detect(self, px: np.ndarray, py: np.ndarray, expect=None, entry_only: bool = False):
        """Find the pallet. Returns a Detection (pallet centre and axis angle, robot frame) or None.

        expect      pallet pose currently being tracked (robot frame), or None while searching.
                    When tracking, only points around the tracked pallet are considered.
        entry_only  accept only detections derived from an entry edge (used during alignment,
                    where the entry-edge centre is the thing being steered at)
        """
        if expect is not None:
            near = (px - expect[0]) ** 2 + (py - expect[1]) ** 2 < self._gate ** 2
            px, py = px[near], py[near]
        edges = self.detect_edges(px, py)
        sx, sy = self.sensor_xy
        things = [e for e in edges if e.kind != "wall" and e.length < self._wall_len]
        self.blob = min((e.pose[:2] for e in things), key=lambda q: math.hypot(q[0] - sx, q[1] - sy), default=None)

        found = []
        for e in edges:
            ex, ey, eyaw = e.pose                        # edge centre, x axis into the pallet
            if e.kind == "entry":
                h = 0.5 * self.pallet.depth              # pallet centre is half a length behind an entry edge
                found.append(Detection((ex + h * math.cos(eyaw), ey + h * math.sin(eyaw), eyaw), "entry", e))
            elif e.kind == "side" and not entry_only:
                h = 0.5 * self.pallet.width              # ... and half a width behind a side edge
                found.append(Detection((ex + h * math.cos(eyaw), ey + h * math.sin(eyaw),
                                        wrap(eyaw - 0.5 * math.pi)), "side", e))
        if expect is not None:
            found = [d for d in found
                     if abs(wrap_half(d.pallet[2] - expect[2])) < math.radians(20.0)
                     and math.hypot(d.pallet[0] - expect[0], d.pallet[1] - expect[1]) < 0.35]
        if not found:
            return None
        # an entry edge pins the pallet down better than a side edge; otherwise nearest first
        return min(found, key=lambda d: (d.source != "entry", math.hypot(d.pallet[0] - sx, d.pallet[1] - sy)))


class OraclePerception:
    """Ground truth plus Gaussian noise. Lets control engineers dial in a detection
    error directly without running the lidar pipeline."""

    def __init__(self, pos_sigma: float, yaw_sigma_deg: float, rng):
        self.pos_sigma, self.yaw_sigma, self.rng = pos_sigma, math.radians(yaw_sigma_deg), rng
        self.truth_robot = None   # true pallet pose in the robot frame, set by the trial runner every tick
        self.last_edges, self.blob = [], None

    def detect(self, px, py, expect=None, entry_only: bool = False):
        x, y, yaw = self.truth_robot
        z = self.rng.standard_normal(3)
        return Detection((x + self.pos_sigma * z[0], y + self.pos_sigma * z[1], yaw + self.yaw_sigma * z[2]),
                         "entry", None)
