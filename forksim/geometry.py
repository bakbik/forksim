"""2D pose maths and ray casting. A pose is a plain tuple (x, y, heading)."""
from __future__ import annotations

import math

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_half(a: float) -> float:
    """Wrap to +/-90 deg: for directions that are the same when flipped by 180 deg."""
    return (a + 0.5 * math.pi) % math.pi - 0.5 * math.pi


def compose(a, b):
    """Pose b, given in frame a, expressed in a's parent frame."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(ath + bth))


def inverse(a):
    ax, ay, ath = a
    c, s = math.cos(ath), math.sin(ath)
    return (-(c * ax + s * ay), s * ax - c * ay, -ath)


def relative(a, b):
    """Pose b expressed in frame a (both given in the same parent frame)."""
    return compose(inverse(a), b)


def rect_segments(x0, x1, y0, y1, pose) -> list:
    """Four wall segments [ax, ay, bx, by] of an axis-aligned rectangle in `pose`'s frame."""
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    world = [compose(pose, (cx, cy, 0.0))[:2] for cx, cy in corners]
    return [[*world[i], *world[(i + 1) % 4]] for i in range(4)]


def raycast(origin, angles: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """Distance along each ray (world-frame angle) to the nearest segment, inf if none."""
    px, py = origin
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    ex, ey = (segs[:, 2] - segs[:, 0])[None, :], (segs[:, 3] - segs[:, 1])[None, :]
    wx, wy = (segs[:, 0] - px)[None, :], (segs[:, 1] - py)[None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        denom = dx * ey - dy * ex
        t = (wx * ey - wy * ex) / denom
        u = (wx * dy - wy * dx) / denom
    hit = (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
    return np.where(hit, t, np.inf).min(axis=1)
