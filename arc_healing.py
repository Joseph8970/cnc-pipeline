"""
arc_healing.py
==============
Arc Healing stage for dxf_normalizer.py.

OpenCutList sometimes exports one smooth SketchUp arc as a fragmented
sequence of DXF ARC entities separated by tiny LINE connectors.  This
module detects those chains and reconstructs them as a single DXF ARC
(or CIRCLE if the sweep is ≈ 360°) using algebraic least-squares circle
fitting.

Contract
--------
* Called once per DXF file, covering modelspace + all block definitions.
* Runs BEFORE any routing / layer processing in the normalizer.
* Does NOT touch LWPOLYLINEs, CIRCLEs, SPLINEs, or any other entity type.
* If confidence is low the original geometry is left completely unchanged.
* False-negative (miss a bad chain) is acceptable.
* False-positive (corrupt good geometry) is NOT acceptable.

Public API
----------
    report = heal_arcs_in_doc(doc, cfg=None)
    print_arc_heal_report(report, filename="")
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

class ArcHealConfig:
    """All tunable tolerances for the arc healing algorithm."""
    max_fit_error_mm:       float = 0.05   # max point-to-circle residual
    max_connector_mm:       float = 0.25   # LINE shorter than this is a bridge artifact
    endpoint_tol_mm:        float = 0.05   # snap tolerance for endpoint matching
    min_sweep_deg:          float = 5.0    # skip chain if reconstructed sweep < this
    circle_closure_tol_deg: float = 1.0    # remaining gap < this → output CIRCLE
    max_sweep_deviation_deg: float = 45.0  # geometric vs summed sweep discrepancy limit


# Default config instance used when the caller passes cfg=None
_DEFAULT_CFG = ArcHealConfig()


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _arc_pt(arc, angle_deg: float) -> Tuple[float, float]:
    cx = arc.dxf.center.x
    cy = arc.dxf.center.y
    r  = arc.dxf.radius
    a  = math.radians(angle_deg)
    return (cx + r * math.cos(a), cy + r * math.sin(a))


def _arc_start(arc) -> Tuple[float, float]:
    return _arc_pt(arc, arc.dxf.start_angle)


def _arc_end(arc) -> Tuple[float, float]:
    return _arc_pt(arc, arc.dxf.end_angle)


def _arc_mid(arc) -> Tuple[float, float]:
    sa    = arc.dxf.start_angle
    sweep = (arc.dxf.end_angle - sa) % 360.0
    return _arc_pt(arc, sa + sweep / 2.0)


def _arc_sweep_deg(arc) -> float:
    """CCW sweep of an ARC entity in degrees, in [0, 360)."""
    return (arc.dxf.end_angle - arc.dxf.start_angle) % 360.0


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _entity_endpoints(
    e,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    t = e.dxftype()
    if t == "ARC":
        return _arc_start(e), _arc_end(e)
    if t == "LINE":
        s = e.dxf.start
        f = e.dxf.end
        return (s.x, s.y), (f.x, f.y)
    return None


def _entity_chord(e) -> float:
    ep = _entity_endpoints(e)
    if not ep:
        return float("inf")
    return _dist(ep[0], ep[1])


def _arc_sample_points(arc) -> List[Tuple[float, float]]:
    """Sample points along *arc* for circle fitting (more for longer arcs)."""
    sa    = arc.dxf.start_angle
    sweep = _arc_sweep_deg(arc)
    pts   = [_arc_start(arc), _arc_end(arc), _arc_mid(arc)]
    if sweep > 60.0:
        pts.append(_arc_pt(arc, sa + sweep * 0.25))
        pts.append(_arc_pt(arc, sa + sweep * 0.75))
    return pts


# ---------------------------------------------------------------------------
# Least-squares algebraic circle fitting
# ---------------------------------------------------------------------------

def _fit_circle(
    pts: List[Tuple[float, float]],
) -> Optional[Tuple[float, float, float]]:
    """
    Algebraic least-squares circle fit (no external dependencies).

    Minimises  Σ(xi² + yi² + A·xi + B·yi + C)²
    giving circle  x² + y² + Ax + By + C = 0
    with centre (-A/2, -B/2) and radius √(A²/4 + B²/4 – C).

    Returns (cx, cy, radius) or None if the system is degenerate.

    Points are centroid-normalised before solving for numerical stability.
    """
    n = len(pts)
    if n < 3:
        return None

    # Centroid normalisation
    cx0 = sum(x for x, y in pts) / n
    cy0 = sum(y for x, y in pts) / n
    npts = [(x - cx0, y - cy0) for x, y in pts]

    sx = sy = sxx = syy = sxy = 0.0
    b1 = b2 = b3 = 0.0
    for x, y in npts:
        r2  = x * x + y * y
        sx  += x;    sy  += y
        sxx += x * x;  syy += y * y;  sxy += x * y
        b1  -= x * r2
        b2  -= y * r2
        b3  -= r2

    fn = float(n)

    def det3(a00, a01, a02,
             a10, a11, a12,
             a20, a21, a22) -> float:
        return (a00 * (a11 * a22 - a12 * a21)
              - a01 * (a10 * a22 - a12 * a20)
              + a02 * (a10 * a21 - a11 * a20))

    D = det3(sxx, sxy, sx,
             sxy, syy, sy,
             sx,  sy,  fn)
    if abs(D) < 1e-12:
        return None

    A = det3(b1,  sxy, sx,
             b2,  syy, sy,
             b3,  sy,  fn) / D

    B = det3(sxx, b1,  sx,
             sxy, b2,  sy,
             sx,  b3,  fn) / D

    C = det3(sxx, sxy, b1,
             sxy, syy, b2,
             sx,  sy,  b3) / D

    r2 = A * A / 4.0 + B * B / 4.0 - C
    if r2 <= 0.0:
        return None

    return (-A / 2.0 + cx0, -B / 2.0 + cy0, math.sqrt(r2))


def _max_residual(
    pts: List[Tuple[float, float]],
    cx: float,
    cy: float,
    r: float,
) -> float:
    return max(abs(math.hypot(x - cx, y - cy) - r) for x, y in pts)


# ---------------------------------------------------------------------------
# Chain building
# ---------------------------------------------------------------------------

def _build_chains(
    entities: list,
    endpoint_tol: float,
    max_connector_mm: float,
) -> List[List]:
    """
    Find connected linear chains of ARC entities (possibly bridged by tiny
    LINE connectors).

    Rules:
    - Consecutive entities share an endpoint within *endpoint_tol*.
    - A LINE in the chain must have chord < *max_connector_mm*.
    - A branch point (multiple entities starting at the same snap key)
      terminates the chain — conservative by design.
    - All ARC entities in a chain must share the same layer.
    - Only chains with ≥ 2 ARC entities are returned.
    """
    # Collect eligible entities
    eligible = []
    for e in entities:
        if e.dxftype() in ("ARC", "LINE") and _entity_endpoints(e) is not None:
            eligible.append(e)

    if len(eligible) < 2:
        return []

    inv_tol = 1.0 / max(endpoint_tol, 1e-9)

    def snap(x: float, y: float) -> Tuple[int, int]:
        return (round(x * inv_tol), round(y * inv_tol))

    # Build start-point → [entity] map
    start_map: Dict[Tuple[int, int], List] = {}
    for e in eligible:
        sp, _ = _entity_endpoints(e)  # type: ignore[misc]
        start_map.setdefault(snap(*sp), []).append(e)

    # Build successor map: id(e) → e_next (only when unambiguous)
    succ: Dict[int, object] = {}
    for e in eligible:
        _, ep = _entity_endpoints(e)  # type: ignore[misc]
        candidates = [c for c in start_map.get(snap(*ep), []) if id(c) != id(e)]
        if len(candidates) != 1:
            continue  # branch or dead-end — do not chain

        nxt = candidates[0]

        # A LINE connector must be tiny
        if nxt.dxftype() == "LINE" and _entity_chord(nxt) >= max_connector_mm:
            continue

        # All ARCs must share the same layer
        if nxt.dxftype() == "ARC" and e.dxftype() == "ARC":
            if getattr(nxt.dxf, "layer", "") != getattr(e.dxf, "layer", ""):
                continue

        succ[id(e)] = nxt

    # Identify heads: no other entity points to them
    has_pred: set = set()
    for nxt in succ.values():
        has_pred.add(id(nxt))

    used: set = set()
    chains: List[List] = []

    def walk(start_e) -> List:
        chain = [start_e]
        used.add(id(start_e))
        curr = start_e
        while id(curr) in succ:
            nxt = succ[id(curr)]
            if id(nxt) in used:
                break
            chain.append(nxt)
            used.add(id(nxt))
            curr = nxt
        return chain

    # Walk from heads first
    for e in eligible:
        if id(e) not in has_pred and id(e) not in used:
            chain = walk(e)
            if sum(1 for c in chain if c.dxftype() == "ARC") >= 2:
                chains.append(chain)

    # Pick up any remaining chains (closed loops with no head)
    for e in eligible:
        if id(e) not in used:
            chain = walk(e)
            if sum(1 for c in chain if c.dxftype() == "ARC") >= 2:
                chains.append(chain)

    return chains


# ---------------------------------------------------------------------------
# DXF attribute preservation
# ---------------------------------------------------------------------------

def _copy_attribs(src) -> dict:
    """Return a dxfattribs dict preserving layer, color, linetype from *src*."""
    attribs: dict = {}
    try:
        attribs["layer"] = src.dxf.layer
    except Exception:
        pass
    try:
        if src.dxf.hasattr("color"):
            attribs["color"] = src.dxf.color
    except Exception:
        pass
    try:
        if src.dxf.hasattr("linetype"):
            attribs["linetype"] = src.dxf.linetype
    except Exception:
        pass
    return attribs


# ---------------------------------------------------------------------------
# Per-layout healing pass
# ---------------------------------------------------------------------------

def _heal_layout(layout, cfg: ArcHealConfig) -> dict:
    """
    Detect and repair fragmented arc chains in a single layout (modelspace
    or block definition).  Returns a partial report dict.
    """
    report = {
        "chains_detected": 0,
        "chains_repaired": 0,
        "circles_reconstructed": 0,
        "chains_skipped": 0,
        "max_fitting_error_mm": 0.0,
    }

    entities = [e for e in layout if e.dxftype() in ("ARC", "LINE")]
    chains   = _build_chains(entities, cfg.endpoint_tol_mm, cfg.max_connector_mm)
    report["chains_detected"] = len(chains)

    for chain in chains:
        arcs = [e for e in chain if e.dxftype() == "ARC"]

        # ---- Collect sample points from ARC entities only ---------------
        sample_pts: List[Tuple[float, float]] = []
        for arc in arcs:
            sample_pts.extend(_arc_sample_points(arc))

        # Include LINE connector endpoints (they must also lie on the circle)
        for e in chain:
            if e.dxftype() == "LINE":
                ep = _entity_endpoints(e)
                if ep:
                    sample_pts.extend(ep)

        # Deduplicate to 0.001 mm
        seen: set = set()
        unique_pts: List[Tuple[float, float]] = []
        for pt in sample_pts:
            key = (round(pt[0] * 1000), round(pt[1] * 1000))
            if key not in seen:
                seen.add(key)
                unique_pts.append(pt)
        sample_pts = unique_pts

        if len(sample_pts) < 3:
            report["chains_skipped"] += 1
            continue

        # ---- Least-squares circle fit -----------------------------------
        fit = _fit_circle(sample_pts)
        if fit is None:
            report["chains_skipped"] += 1
            continue

        cx, cy, r = fit

        # ---- Confidence check: geometric residual -----------------------
        max_err = _max_residual(sample_pts, cx, cy, r)
        report["max_fitting_error_mm"] = max(
            report["max_fitting_error_mm"], max_err
        )

        if max_err > cfg.max_fit_error_mm:
            report["chains_skipped"] += 1
            continue

        # ---- Determine chain endpoints and projected angles -------------
        chain_start = _entity_endpoints(chain[0])[0]   # type: ignore[index]
        chain_end   = _entity_endpoints(chain[-1])[1]   # type: ignore[index]

        sa_deg = math.degrees(
            math.atan2(chain_start[1] - cy, chain_start[0] - cx)
        ) % 360.0
        ea_deg = math.degrees(
            math.atan2(chain_end[1] - cy, chain_end[0] - cx)
        ) % 360.0

        # CCW sweep from sa_deg to ea_deg
        computed_sweep = (ea_deg - sa_deg) % 360.0

        # ---- Sanity: summed arc sweeps should match computed sweep ------
        total_arc_sweep = sum(_arc_sweep_deg(a) for a in arcs)

        chain_gap   = _dist(chain_start, chain_end)
        is_circle   = (
            chain_gap < cfg.endpoint_tol_mm
            or (360.0 - computed_sweep) < cfg.circle_closure_tol_deg
        )

        if not is_circle:
            # Sweep deviation check (skip near-360 ambiguity for circle case)
            if abs(computed_sweep - total_arc_sweep) > cfg.max_sweep_deviation_deg:
                report["chains_skipped"] += 1
                continue

            if computed_sweep < cfg.min_sweep_deg:
                report["chains_skipped"] += 1
                continue

        # ---- Replace chain with single entity ---------------------------
        attribs = _copy_attribs(arcs[0])

        for e in chain:
            try:
                e.destroy()
            except Exception:
                pass

        if is_circle:
            layout.add_circle(
                center=(cx, cy, 0.0),
                radius=r,
                dxfattribs=attribs,
            )
            report["circles_reconstructed"] += 1
        else:
            layout.add_arc(
                center=(cx, cy, 0.0),
                radius=r,
                start_angle=sa_deg,
                end_angle=ea_deg,
                dxfattribs=attribs,
            )

        report["chains_repaired"] += 1

    return report


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def heal_arcs_in_doc(doc, cfg: ArcHealConfig = None) -> dict:
    """
    Apply arc healing to modelspace and every block definition in *doc*.

    Parameters
    ----------
    doc : ezdxf document
    cfg : ArcHealConfig or None (uses module defaults)

    Returns
    -------
    Aggregated report dict with keys:
        chains_detected, chains_repaired, circles_reconstructed,
        chains_skipped, max_fitting_error_mm
    """
    if cfg is None:
        cfg = _DEFAULT_CFG

    total: dict = {
        "chains_detected": 0,
        "chains_repaired": 0,
        "circles_reconstructed": 0,
        "chains_skipped": 0,
        "max_fitting_error_mm": 0.0,
    }

    def _merge(r: dict) -> None:
        for k in ("chains_detected", "chains_repaired",
                  "circles_reconstructed", "chains_skipped"):
            total[k] += r[k]
        total["max_fitting_error_mm"] = max(
            total["max_fitting_error_mm"], r["max_fitting_error_mm"]
        )

    _merge(_heal_layout(doc.modelspace(), cfg))
    for name in doc.blocks.block_names():
        try:
            blk = doc.blocks.get(name)
            _merge(_heal_layout(blk, cfg))
        except Exception:
            pass

    return total


def print_arc_heal_report(report: dict, filename: str = "") -> None:
    """Print the Arc Healing Report to stdout."""
    label = f" [{filename}]" if filename else ""
    detected  = report["chains_detected"]
    repaired  = report["chains_repaired"]
    circles   = report["circles_reconstructed"]
    skipped   = report["chains_skipped"]
    max_err   = report["max_fitting_error_mm"]

    print(f"\n--- Arc Healing Report{label} ---")
    print(f"  Arc chains detected:   {detected}")
    print(f"  Arc chains repaired:   {repaired}")
    print(f"  Circles reconstructed: {circles}")
    print(f"  Chains skipped:        {skipped}")
    if detected > 0:
        print(f"  Max fitting error:     {max_err:.4f} mm")
    print("---")
