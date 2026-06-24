"""
AUTO CONCAVE HOLE GENERATOR
-------------------------------------------
Detects concave corners in CLOSED LWPOLYLINE contours
and inserts Ø5 mm drilling holes on V_DrillSF_19.1.

Works on normalized DXFs produced by dxf_normalizer.py:
geometry lives inside OCL_PART___ block definitions,
not directly in modelspace.

Public API
----------
    holes_added = process_file(in_path, out_path)

Can also be run as a standalone script:
    python auto_concave_holes.py input.dxf output.dxf
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Tuple

import ezdxf
from ezdxf.math import Vec2

# ── constants ────────────────────────────────────────────────────────────────
HOLE_DIAMETER = 5.0
HOLE_RADIUS   = HOLE_DIAMETER / 2.0
HOLE_LAYER    = "V_DrillSF_19.1"

# Only add holes to contours on these layers (outer contour + pocket contours).
# Empty set = process every closed LWPOLYLINE.
TARGET_LAYERS: set[str] = set()   # extend if you want layer filtering


# ── geometry helpers ─────────────────────────────────────────────────────────

def _signed_area(pts: List[Vec2]) -> float:
    n   = len(pts)
    acc = 0.0
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        acc += x1 * y2 - x2 * y1
    return acc


def _concave_vertices(pts: List[Vec2]) -> List[Vec2]:
    """Return vertices that form a concave (reflex) corner."""
    if len(pts) < 3:
        return []
    ccw     = _signed_area(pts) > 0
    result  = []
    n       = len(pts)
    for i in range(n):
        a = pts[(i - 1) % n]
        b = pts[i]
        c = pts[(i + 1) % n]
        ab = b - a
        bc = c - b
        cross = ab.x * bc.y - ab.y * bc.x
        if (ccw and cross < 0) or (not ccw and cross > 0):
            result.append(b)
    return result


# ── core processing ──────────────────────────────────────────────────────────

def _ensure_layer(doc: ezdxf.document.Drawing) -> None:
    if HOLE_LAYER not in doc.layers:
        doc.layers.add(name=HOLE_LAYER, color=4)   # turquoise = blind drill


def _process_layout(layout, doc: ezdxf.document.Drawing) -> int:
    """Add concave-corner holes to all eligible closed LWPOLYLINEs in *layout*.
    Returns the number of holes added."""
    _ensure_layer(doc)
    holes = 0

    for ent in layout.query("LWPOLYLINE"):
        if not ent.closed:
            continue
        if TARGET_LAYERS and ent.dxf.layer not in TARGET_LAYERS:
            continue

        pts = [Vec2(*pt[:2]) for pt in ent.get_points()]
        for vertex in _concave_vertices(pts):
            layout.add_circle(
                center=(vertex.x, vertex.y),
                radius=HOLE_RADIUS,
                dxfattribs={"layer": HOLE_LAYER},
            )
            holes += 1

    return holes


def process_file(in_path: Path, out_path: Path) -> int:
    """
    Read *in_path*, add concave-corner holes, save to *out_path*.
    Returns total number of holes inserted (0 = no concave corners found).
    Raises on read/write failure.
    """
    doc   = ezdxf.readfile(str(in_path))
    total = 0

    # Modelspace (handles non-OCL DXFs and any direct geometry)
    total += _process_layout(doc.modelspace(), doc)

    # OCL_PART___ block definitions (normalized DXF structure)
    for name in doc.blocks.block_names():
        if name.upper().startswith("OCL_PART___"):
            blk    = doc.blocks.get(name)
            total += _process_layout(blk, doc)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    doc.saveas(str(out_path))
    return total


# ── standalone CLI ───────────────────────────────────────────────────────────

def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Add concave-corner relief holes to a normalized DXF."
    )
    ap.add_argument("input",  help="Input DXF file")
    ap.add_argument("output", help="Output DXF file")
    args = ap.parse_args()

    in_p  = Path(args.input)
    out_p = Path(args.output)

    if not in_p.exists():
        print(f"[ERROR] Input not found: {in_p}", file=sys.stderr)
        return 1

    print(f"[INFO] Processing: {in_p.name}")
    n = process_file(in_p, out_p)
    if n:
        print(f"[OK] {n} concave-corner hole(s) added -> {out_p}")
    else:
        print(f"[OK] No concave corners found; file copied as-is -> {out_p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
