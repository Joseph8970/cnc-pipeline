"""
AUTO CONCAVE HOLE GENERATOR
-------------------------------------------
Detects interlocking-notch shoulder corners in CLOSED LWPOLYLINE contours
and inserts Ø5 mm drilling holes on V_DrillSF_19.1.

Works on normalized DXFs produced by dxf_normalizer.py:
geometry lives inside OCL_PART___ block definitions,
not directly in modelspace.

===========================================================================
ALGORITHM — interior-facing concave corner
===========================================================================
For every vertex B (with neighbours A and C):

  1. Determine whether B is concave (reflex).
     If not → skip.

  2. Compute normalised edge vectors from B toward its neighbours:
       v1 = normalise(A − B)
       v2 = normalise(C − B)

  3. Compute the angle bisector:
       bisector = normalise(v1 + v2)

  4. Probe 0.5 mm along the bisector:
       test_point = B + bisector × 0.5

  5. Run a point-in-polygon test on test_point against the LWPOLYLINE.
     • INSIDE  → corner faces the part interior → create relief hole.
     • OUTSIDE → corner faces a void/notch opening → skip.

  6. Hole centre is placed tangent to both walls:
       hole_centre = B + bisector × HOLE_RADIUS

This is fully geometry-driven: no axis assumptions, no orientation
assumptions, works for CW and CCW polygons, any rotation or mirror.

Public API
----------
    holes_added = process_file(in_path, out_path)

Can also be run as a standalone script:
    python auto_concave_holes.py input.dxf output.dxf
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Tuple

import ezdxf
from ezdxf.math import Vec2

# ── constants ────────────────────────────────────────────────────────────────
HOLE_DIAMETER  = 5.0
HOLE_RADIUS    = HOLE_DIAMETER / 2.0
HOLE_LAYER     = "V_DrillSF_19.1"

# Distance along the bisector used for the interior probe.
PROBE_DIST_MM  = 0.5

TARGET_LAYERS: set[str] = set()


# ── geometry helpers ─────────────────────────────────────────────────────────

def _signed_area(pts: List[Vec2]) -> float:
    n = len(pts)
    acc = 0.0
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        acc += x1 * y2 - x2 * y1
    return acc


def _point_in_polygon(px: float, py: float, pts: List[Vec2]) -> bool:
    """Ray-casting point-in-polygon test (works for CW and CCW polygons)."""
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i].x, pts[i].y
        xj, yj = pts[j].x, pts[j].y
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


MAX_STRAIGHT_BULGE = 0.01   # bulge threshold: straight segment vs arc segment


def _relief_holes(ent) -> List[Vec2]:
    """
    Return hole-centre positions for every milling-concave corner of a closed
    LWPOLYLINE, using a bisector + point-in-polygon test.

    Milling-concave means the inside corner of a notch/pocket — the spot the
    router bit cannot reach cleanly, requiring a relief hole.

    Key insight: the bisector at a milling-concave corner always points toward
    the notch void.  Whether that void reads as 'inside' or 'outside' the polygon
    depends on winding:
      • CW polygon  — the notch void is enclosed (inside)  → probe IS inside
      • CCW polygon — the notch void is excluded (outside) → probe is NOT inside
    The unified condition is therefore:  probe_inside XOR ccw
    i.e., skip if (ccw == probe_inside).

    Arc-profile junction vertices (non-zero bulge on an adjacent segment) are
    excluded before the PIP test to avoid false positives on curved profiles.
    """
    raw = list(ent.get_points("xyb"))
    if len(raw) < 3:
        return []

    pts    = [Vec2(float(p[0]), float(p[1])) for p in raw]
    bulges = [float(p[2]) for p in raw]
    n      = len(pts)
    ccw    = _signed_area(pts) > 0
    result: List[Vec2] = []

    for i in range(n):
        a = pts[(i - 1) % n]
        b = pts[i]
        c = pts[(i + 1) % n]

        ab    = b - a
        bc    = c - b
        cross = ab.x * bc.y - ab.y * bc.x

        # ── 1. milling-concave corners always have cross < 0 ──────────
        #    (topologically concave for CCW; topologically convex for CW —
        #     both give cross < 0 for the inner corner of a pocket)
        if cross >= 0:
            continue

        # ── 1b. both adjacent segments must be straight ────────────────
        in_b  = bulges[(i - 1) % n]
        out_b = bulges[i]
        if abs(in_b) > MAX_STRAIGHT_BULGE or abs(out_b) > MAX_STRAIGHT_BULGE:
            continue

        # ── 2. edge vectors from B toward neighbours ───────────────────
        v1x, v1y = a.x - b.x, a.y - b.y
        v2x, v2y = c.x - b.x, c.y - b.y
        l1 = math.hypot(v1x, v1y)
        l2 = math.hypot(v2x, v2y)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        v1x /= l1;  v1y /= l1
        v2x /= l2;  v2y /= l2

        # ── 3. angle bisector ──────────────────────────────────────────
        bsx, bsy = v1x + v2x, v1y + v2y
        bl = math.hypot(bsx, bsy)
        if bl < 1e-9:
            continue          # 180° angle — degenerate
        bsx /= bl;  bsy /= bl

        # ── 4. probe point ─────────────────────────────────────────────
        tx = b.x + bsx * PROBE_DIST_MM
        ty = b.y + bsy * PROBE_DIST_MM

        # ── 5. interior test ───────────────────────────────────────────
        #    For CW polygon:  notch void is enclosed  → probe_inside must be True
        #    For CCW polygon: notch void is excluded  → probe_inside must be False
        #    Unified: skip if (ccw == probe_inside)
        probe_inside = _point_in_polygon(tx, ty, pts)
        if ccw == probe_inside:
            continue

        # ── 6. hole centre tangent to both walls ───────────────────────
        result.append(Vec2(b.x + bsx * HOLE_RADIUS,
                           b.y + bsy * HOLE_RADIUS))

    return result


# ── core processing ──────────────────────────────────────────────────────────

def _ensure_layer(doc) -> None:
    if HOLE_LAYER not in doc.layers:
        doc.layers.add(name=HOLE_LAYER, color=4)


def _process_layout(layout, doc) -> int:
    _ensure_layer(doc)
    holes = 0
    for ent in layout.query("LWPOLYLINE"):
        if not ent.closed:
            continue
        if TARGET_LAYERS and ent.dxf.layer not in TARGET_LAYERS:
            continue
        for center in _relief_holes(ent):
            layout.add_circle(
                center=(center.x, center.y),
                radius=HOLE_RADIUS,
                dxfattribs={"layer": HOLE_LAYER},
            )
            holes += 1
    return holes


def process_file(in_path: Path, out_path: Path) -> int:
    doc   = ezdxf.readfile(str(in_path))
    total = 0

    total += _process_layout(doc.modelspace(), doc)

    for name in doc.blocks.block_names():
        if name.upper().startswith("OCL_PART___"):
            total += _process_layout(doc.blocks.get(name), doc)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    doc.saveas(str(out_path))
    return total


# ── standalone CLI ───────────────────────────────────────────────────────────

def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Add notch-shoulder relief holes to a normalized DXF."
    )
    ap.add_argument("input",  help="Input DXF file")
    ap.add_argument("output", help="Output DXF file")
    args = ap.parse_args()

    in_p  = Path(args.input)
    out_p = Path(args.output)

    if not in_p.exists():
        print(f"[ERROR] Input not found: {in_p}", file=sys.stderr)
        return 1

    n = process_file(in_p, out_p)
    if n:
        print(f"[OK] {n} relief hole(s) added -> {out_p}")
    else:
        print(f"[OK] No interior-facing concave corners found -> {out_p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
