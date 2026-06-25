"""
arc_repair.py — OpenCutList fragmented-arc repair  (v1.0)

OpenCutList (SketchUp plug-in) sometimes exports a single smooth SketchUp arc
as multiple consecutive DXF ARC entities — because SketchUp tessellates arcs
into individual edges and OCL serialises each edge separately.

All fragments share exactly the same center and radius (taken directly from
SketchUp's ArcCurve object).  They connect end-to-end in definition order.
Occasionally a degenerate short LINE entity appears at a seam.

PHASE 1  analyze_arc_chains(doc)   — print diagnostic report (no side effects)
PHASE 2  repair_opencutlist_arcs(doc) — merge in-place, return count merged

Both phases operate on the raw ezdxf document, before any geometry conversion,
routing or layer mapping.
"""

from __future__ import annotations

import math
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Default tolerances  (all in drawing units; the doc is in mm at call time)
# ---------------------------------------------------------------------------
_CENTER_TOL      = 0.05    # mm  — centers must agree within this
_RADIUS_REL_TOL  = 0.005   # 0.5 % — relative radius difference
_GAP_TOL         = 0.05    # mm  — allowed endpoint gap between consecutive pieces
_TINY_LINE_MM    = 1.0     # mm  — connector LINE shorter than this is absorbed


# ---------------------------------------------------------------------------
# Geometry helpers on raw ezdxf ARC / LINE entities
# ---------------------------------------------------------------------------

def _arc_start(arc) -> Tuple[float, float]:
    cx, cy = float(arc.dxf.center.x), float(arc.dxf.center.y)
    r  = float(arc.dxf.radius)
    a  = math.radians(float(arc.dxf.start_angle))
    return (cx + r * math.cos(a), cy + r * math.sin(a))


def _arc_end(arc) -> Tuple[float, float]:
    cx, cy = float(arc.dxf.center.x), float(arc.dxf.center.y)
    r  = float(arc.dxf.radius)
    a  = math.radians(float(arc.dxf.end_angle))
    return (cx + r * math.cos(a), cy + r * math.sin(a))


def _arc_sweep(arc) -> float:
    """CCW sweep angle in degrees, range [0, 360)."""
    sa = float(arc.dxf.start_angle) % 360.0
    ea = float(arc.dxf.end_angle)   % 360.0
    return (ea - sa) % 360.0


def _dist(p: Tuple[float, float], q: Tuple[float, float]) -> float:
    return math.hypot(q[0] - p[0], q[1] - p[1])


def _line_len(line) -> float:
    sx, sy = float(line.dxf.start.x), float(line.dxf.start.y)
    ex, ey = float(line.dxf.end.x),   float(line.dxf.end.y)
    return math.hypot(ex - sx, ey - sy)


def _line_start(line) -> Tuple[float, float]:
    return (float(line.dxf.start.x), float(line.dxf.start.y))


def _line_end(line) -> Tuple[float, float]:
    return (float(line.dxf.end.x), float(line.dxf.end.y))


def _cocircular(a1, a2,
                center_tol:    float = _CENTER_TOL,
                radius_rel_tol: float = _RADIUS_REL_TOL) -> bool:
    r1 = float(a1.dxf.radius)
    r2 = float(a2.dxf.radius)
    avg = (r1 + r2) / 2.0
    if abs(r1 - r2) > radius_rel_tol * avg:
        return False
    cx1, cy1 = float(a1.dxf.center.x), float(a1.dxf.center.y)
    cx2, cy2 = float(a2.dxf.center.x), float(a2.dxf.center.y)
    return math.hypot(cx2 - cx1, cy2 - cy1) <= center_tol


# ---------------------------------------------------------------------------
# Chain data class
# ---------------------------------------------------------------------------

class _Chain:
    __slots__ = ("arcs", "connectors")

    def __init__(self) -> None:
        self.arcs:       List = []   # ordered ARC entities
        self.connectors: List = []   # absorbed tiny LINE entities


# ---------------------------------------------------------------------------
# Greedy forward scan — build chains within one layout
# ---------------------------------------------------------------------------

def _find_chains(layout,
                 gap_tol:        float,
                 tiny_line_mm:   float,
                 center_tol:     float,
                 radius_rel_tol: float) -> List[_Chain]:
    """
    Walk entities in definition order.  Recognises:

      ARC → ARC               (direct, endpoint-match + co-circular)
      ARC → tiny LINE → ARC   (connector absorbed, both arcs co-circular)

    Returns chains with len(arcs) >= 2.
    """
    # Collect only ARC and LINE entities in the order they appear
    entities = [e for e in layout if e.dxftype() in ("ARC", "LINE")]

    chains:   List[_Chain] = []
    consumed: set          = set()   # id(entity) of already-merged entities

    i = 0
    while i < len(entities):
        e = entities[i]

        if e.dxftype() != "ARC" or id(e) in consumed:
            i += 1
            continue

        chain = _Chain()
        chain.arcs.append(e)
        j = i + 1

        while j < len(entities):
            nxt      = entities[j]
            nxt_type = nxt.dxftype()

            # ── direct arc continuation ─────────────────────────────────
            if nxt_type == "ARC" and id(nxt) not in consumed:
                last_arc = chain.arcs[-1]
                gap = _dist(_arc_end(last_arc), _arc_start(nxt))
                if (gap <= gap_tol
                        and _cocircular(last_arc, nxt, center_tol, radius_rel_tol)):
                    chain.arcs.append(nxt)
                    j += 1
                    continue
                break

            # ── connector LINE between two arcs ──────────────────────────
            elif nxt_type == "LINE":
                if _line_len(nxt) > tiny_line_mm:
                    break
                # There must be an arc after the line
                if j + 1 >= len(entities) or entities[j + 1].dxftype() != "ARC":
                    break
                after_arc = entities[j + 1]
                if id(after_arc) in consumed:
                    break
                # The line must bridge: last_arc.end → after_arc.start
                # (accept either direction of the LINE entity)
                last_end   = _arc_end(chain.arcs[-1])
                ls, le     = _line_start(nxt), _line_end(nxt)
                next_start = _arc_start(after_arc)
                forward  = (_dist(last_end, ls) <= gap_tol
                            and _dist(le, next_start) <= gap_tol)
                backward = (_dist(last_end, le) <= gap_tol
                            and _dist(ls, next_start) <= gap_tol)
                if not (forward or backward):
                    break
                if not _cocircular(chain.arcs[-1], after_arc,
                                   center_tol, radius_rel_tol):
                    break
                chain.connectors.append(nxt)
                chain.arcs.append(after_arc)
                j += 2
                continue

            break   # any other entity type stops the chain

        if len(chain.arcs) >= 2:
            for arc  in chain.arcs:       consumed.add(id(arc))
            for line in chain.connectors: consumed.add(id(line))
            chains.append(chain)

        # Advance: if we built a chain, jump to j; otherwise just step
        i = j if len(chain.arcs) >= 2 else i + 1

    return chains


# ---------------------------------------------------------------------------
# PHASE 1 — Analysis  (no side effects)
# ---------------------------------------------------------------------------

def analyze_arc_chains(doc) -> None:
    """
    Print a diagnostic report of every ARC chain found in the document.
    Call this before repair to understand the export pattern.
    """
    print("=" * 62)
    print("ARC CHAIN ANALYSIS — OpenCutList DXF")
    print("=" * 62)

    def _report(name: str, layout) -> None:
        arcs   = list(layout.query("ARC"))
        lines  = list(layout.query("LINE"))
        circs  = list(layout.query("CIRCLE"))
        lwpoly = list(layout.query("LWPOLYLINE"))
        poly   = list(layout.query("POLYLINE"))

        # Skip layouts with no relevant content
        if not arcs and not lines and not circs and not lwpoly and not poly:
            return

        print(f"\nBlock: {name!r}")
        print(f"  Counts: {len(arcs)} ARC  {len(lines)} LINE  "
              f"{len(circs)} CIRCLE  {len(lwpoly)} LWPOLYLINE  "
              f"{len(poly)} POLYLINE")

        chains = _find_chains(
            layout,
            gap_tol=_GAP_TOL,
            tiny_line_mm=_TINY_LINE_MM,
            center_tol=_CENTER_TOL,
            radius_rel_tol=_RADIUS_REL_TOL,
        )

        if not chains:
            print("  No mergeable ARC chains found.")
            return

        for ci, chain in enumerate(chains, 1):
            total_sweep = sum(_arc_sweep(a) for a in chain.arcs)
            print(f"\n  Chain #{ci}: {len(chain.arcs)} arcs, "
                  f"{len(chain.connectors)} connector line(s), "
                  f"total sweep ≈ {total_sweep:.2f}°")
            for k, arc in enumerate(chain.arcs):
                cx  = float(arc.dxf.center.x)
                cy  = float(arc.dxf.center.y)
                r   = float(arc.dxf.radius)
                sa  = float(arc.dxf.start_angle)
                ea  = float(arc.dxf.end_angle)
                sw  = _arc_sweep(arc)
                lyr = arc.dxf.layer
                print(f"    [{k}] center=({cx:.4f},{cy:.4f})  "
                      f"r={r:.4f}  "
                      f"{sa:.3f}°→{ea:.3f}°  sweep={sw:.3f}°  "
                      f"layer={lyr!r}")
                if k < len(chain.arcs) - 1:
                    gap = _dist(_arc_end(arc), _arc_start(chain.arcs[k + 1]))
                    print(f"         gap to [{k+1}]: {gap:.5f} mm")
            for k, line in enumerate(chain.connectors):
                print(f"    connector LINE #{k}: length={_line_len(line):.5f} mm")
            print(f"    → will merge: start={chain.arcs[0].dxf.start_angle:.3f}°  "
                  f"end={chain.arcs[-1].dxf.end_angle:.3f}°  "
                  f"layer={chain.arcs[0].dxf.layer!r}")

    _report("*MODEL_SPACE", doc.modelspace())
    for name in sorted(doc.blocks.block_names()):
        if name.startswith("*"):
            continue
        _report(name, doc.blocks.get(name))

    print("\n" + "=" * 62)


# ---------------------------------------------------------------------------
# PHASE 2 — Repair  (in-place)
# ---------------------------------------------------------------------------

def repair_opencutlist_arcs(
    doc,
    center_tol:      float = _CENTER_TOL,
    radius_rel_tol:  float = _RADIUS_REL_TOL,
    gap_tol:         float = _GAP_TOL,
    tiny_line_mm:    float = _TINY_LINE_MM,
) -> int:
    """
    Merge fragmented OpenCutList ARC chains in-place.

    For every block definition and modelspace:
      • Find co-circular ARC chains (consecutive, endpoint-matched).
      • Absorb tiny LINE connectors between arcs.
      • Replace each chain with a single ARC entity.
      • Preserve layer, color, linetype, ltscale, lineweight from the first arc.

    Returns total number of chains merged.
    """
    total = 0

    def _repair_layout(layout) -> int:
        chains = _find_chains(layout, gap_tol, tiny_line_mm,
                              center_tol, radius_rel_tol)
        count = 0
        for chain in chains:
            first = chain.arcs[0]
            last  = chain.arcs[-1]

            # Build dxfattribs — copy inheritable properties from first arc
            attribs: dict = {"layer": first.dxf.layer}
            for attr in ("color", "linetype", "ltscale", "lineweight",
                         "true_color", "color_name", "transparency"):
                try:
                    if first.dxf.hasattr(attr):
                        attribs[attr] = getattr(first.dxf, attr)
                except Exception:
                    pass

            # Center Z-coordinate (preserve it)
            try:
                cz = float(first.dxf.center.z)
            except Exception:
                cz = 0.0

            try:
                layout.add_arc(
                    center      = (float(first.dxf.center.x),
                                   float(first.dxf.center.y),
                                   cz),
                    radius      = float(first.dxf.radius),
                    start_angle = float(first.dxf.start_angle),
                    end_angle   = float(last.dxf.end_angle),
                    dxfattribs  = attribs,
                )
            except Exception as exc:
                print(f"[arc_repair] WARN: could not add merged arc: {exc}")
                continue

            # Delete original fragments + connectors
            for arc in chain.arcs:
                try:
                    arc.destroy()
                except Exception:
                    pass
            for line in chain.connectors:
                try:
                    line.destroy()
                except Exception:
                    pass

            count += 1

        return count

    total += _repair_layout(doc.modelspace())
    for name in doc.blocks.block_names():
        if name.startswith("*"):
            continue
        total += _repair_layout(doc.blocks.get(name))

    if total:
        print(f"[arc_repair] Merged {total} fragmented arc chain(s).")
    return total
