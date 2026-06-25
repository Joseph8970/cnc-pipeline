#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF normalizer for woodWOP pre-processing (FULL v3.12)

Adds MATERIAL detection + auto-naming:
- bin_001.dxf -> 1-<MATERIAL>-SHEET X 1.dxf
- bin_002_to_005.dxf -> 2-<MATERIAL>-SHEET X 4.dxf
  MATERIAL is detected from filename tokens (PLYWOOD, PL-*, WV-*), then TEXT/MTEXT,
  then block attributes; fallback to 'MATERIAL'.

Core features kept:
- Dynamic layers & routing:
  * Ø<=6mm circles -> Ø5;  D≈T -> V_DrillSF_{T} (red),  D<T -> V_DrillSF_{D} (turquoise)
  * Large circles/closed polys: D≈T -> V_Fraes_0T134L; D<T -> V_Fraes_{R}T134L (purple)
- OUTER protected (mapped to V_Fraes_2T134R green) before other routing.
- Dado pockets: closed LWPOLYLINE on OCL_PART___<depth>Z (no suffix) -> V_Fraes_{R}T134L.
- Spline/Ellipse/Classic Polyline -> LWPOLYLINE.
- Poly start vertex insertion on longest >=30mm segment (no break).
- Robust ByLayer & explicit entity painting to match layer ACI (fixes black).
- Purges old OCL_* layers; safe saving with temp/replace.
"""

from __future__ import annotations

import re
import time
from math import hypot
from pathlib import Path
from typing import List, Tuple, Optional

import ezdxf
from ezdxf.entities import Ellipse, Spline
from ezdxf.math import ConstructionEllipse, Matrix44

# ------------------ CONFIG ------------------

THROUGH_TOL_MM = 0.2           # D ≈ T tolerance
FALLBACK_THICKNESS_MM = 18.0   # only if no OUTER token found

# Thresholds (mm)
OPEN_CLOSED_MIN_TOTAL_LEN = 30.0
SEGMENT_MIN_LEN            = 30.0

# Circle normalization
FIVE_MM_DIAM = 5.0
SIX_MM_DIAM  = 6.0

# Patterns
NUMZ_TOKEN_RE   = re.compile(r"(\d+(?:_\d+)?)Z", re.IGNORECASE)               # e.g., 18_700Z
OUTER_LAYER_RE  = re.compile(r"^OCL_PART___+(\d+(?:_\d+)?)Z_OUTER$", re.IGNORECASE)
POCKET_LAYER_RE = re.compile(r"^OCL_PART___+(\d+(?:_\d+)?)Z$", re.IGNORECASE)  # no _OUTER/_HOLES
OCL_OUTER_RE    = re.compile(r"^OCL_PART___.*?_OUTER$", re.IGNORECASE)
OCL_HOLES_RE    = re.compile(r"^OCL_PART___.*?_HOLES$", re.IGNORECASE)
OCL_PREFIX_RE   = re.compile(r"^OCL_PART___", re.IGNORECASE)

# ACI colors
ACI_BLUE      = 5   # ProcPart_T
ACI_RED       = 1   # V_DrillSF_T (through)
ACI_TURQUOISE = 4   # V_DrillSF_D (blind)
ACI_GREEN     = 3   # V_Fraes_2T134R (outer)
ACI_PURPLE    = 6   # V_Fraes_*T134L (pockets & milling)

# ------------------ UTILITIES ------------------

def fmt_val(val: float) -> str:
    """Round to 1 decimal; drop .0 (e.g., 18.74->18.7, 25.0->25)."""
    v = round(float(val), 1)
    if abs(v - int(v)) < 1e-9:
        return str(int(v))
    return f"{v:.1f}"

def is_plywood(material: str) -> bool:
    return (material or "").strip().upper() == "PLYWOOD"


def get_outer_layer(material: str) -> str:
    if is_plywood(material):
        return "V_Fraes_2T137R"
    return "V_Fraes_2T134R"


def get_through_milling_layer(material: str) -> str:
    if is_plywood(material):
        return "V_Fraes_0T137L"
    return "V_Fraes_0T132L"


def get_pocket_layer(depth: float) -> str:
    return f"F_Pocket_{fmt_val(depth)}"

def ensure_layer(doc: ezdxf.EzfDocument, name: Optional[str], aci: Optional[int] = None):
    if not name:
        return
    if name not in doc.layers:
        doc.layers.add(name)
    lyr = doc.layers.get(name)
    # Clear any layer true-color that could force black
    try:
        if hasattr(lyr.dxf, "true_color"):
            del lyr.dxf.true_color
    except Exception:
        pass
    # Assign color if provided; otherwise infer by naming
    try:
        if aci is not None:
            lyr.color = aci
        else:
            if name.startswith("V_Fraes_2T") and name.endswith("R"):
                lyr.color = ACI_GREEN
            elif name.startswith("V_Fraes_"):
                lyr.color = ACI_PURPLE
            elif name.startswith("F_Pocket_"):
                lyr.color = ACI_PURPLE
            elif name.startswith("V_DrillSF_"):
                lyr.color = ACI_RED  # default for through if unspecified
            elif name.startswith("ProcPart_"):
                lyr.color = ACI_BLUE
    except Exception:
        pass

def units_code(doc: ezdxf.EzfDocument) -> int:
    return int(doc.header.get("$INSUNITS", 0) or 0)

def scale_layout(layout, factor: float):
    m = Matrix44.scale(factor, factor, factor)
    for e in list(layout):
        try:
            e.transform(m)
        except Exception:
            pass

def auto_units_to_mm(doc: ezdxf.EzfDocument):
    code = units_code(doc)
    if code == 1:
        print("[INFO] Units: inches detected -> scaling 25.4× and setting to mm.")
        scale_layout(doc.modelspace(), 25.4)
        for name in doc.blocks.block_names():
            blk = doc.blocks.get(name)
            scale_layout(blk, 25.4)
        doc.header["$INSUNITS"] = 4
    else:
        if code != 4:
            print(f"[INFO] Units code {code} -> setting header to mm (no scale).")
        doc.header["$INSUNITS"] = 4

def parse_numz(token: str) -> Optional[float]:
    """Parse '18_700Z' -> 18.700"""
    try:
        return float(token.replace("_", ".").rstrip("Zz"))
    except Exception:
        return None

def parse_depth_from_layer(layer_name: str) -> Optional[float]:
    m = NUMZ_TOKEN_RE.findall(layer_name or "")
    if not m:
        return None
    return parse_numz(m[-1])

def get_material_thickness(doc: ezdxf.EzfDocument) -> float:
    """Scan layer names and read the max thickness from any OUTER token."""
    thicknesses = []
    for layer in doc.layers:
        ln = layer.dxf.name or ""
        hit = OUTER_LAYER_RE.match(ln)
        if hit:
            v = parse_numz(hit.group(1))
            if v is not None:
                thicknesses.append(v)
    if thicknesses:
        return max(thicknesses)
    print(f"[WARN] No OUTER thickness layer found; using fallback {FALLBACK_THICKNESS_MM} mm.")
    return FALLBACK_THICKNESS_MM

# ---------- Effective layer for entities inside blocks ----------

def infer_block_layer_from_inserts(doc: ezdxf.EzfDocument, block_name: str) -> Optional[str]:
    """Return the unique layer used by INSERTs of this block, or None if mixed/none."""
    layers = set()
    for ins in doc.modelspace().query('INSERT'):
        try:
            if ins.dxf.name == block_name and hasattr(ins.dxf, "layer"):
                ln = ins.dxf.layer or ""
                if ln:
                    layers.add(ln)
        except Exception:
            pass
    if len(layers) == 1:
        return next(iter(layers))
    return None

_BLOCK_LAYER_CACHE: dict[str, Optional[str]] = {}

def effective_layer_name(e, layout, doc) -> str:
    """
    Return the layer name to use for parsing depth:
    - entity's own layer if not '0'
    - if inside a BlockLayout AND entity layer is '0', try the unique INSERT layer
    """
    ln = (getattr(e.dxf, "layer", "") or "").strip()
    if ln and ln != "0":
        return ln
    # In blocks, try to infer from INSERTs
    try:
        blk_name = getattr(layout, "name", None) or getattr(layout, "block_name", None)
    except Exception:
        blk_name = None
    if not blk_name:
        return ln or ""
    if blk_name not in _BLOCK_LAYER_CACHE:
        _BLOCK_LAYER_CACHE[blk_name] = infer_block_layer_from_inserts(doc, blk_name)
    return _BLOCK_LAYER_CACHE[blk_name] or (ln or "")

# ---------- CURVE CONVERSION ----------

def ellipse_to_lwpolyline(msp_or_blk, ell: Ellipse, approx_seg=1.0):
    ce = ConstructionEllipse.from_ellipse(ell)
    count = max(24, int(max(ce.major_axis.magnitude, 1.0) / max(approx_seg, 0.1)))
    pts = [ce.point_at(t) for t in [i / count for i in range(count + 1)]]
    lw = msp_or_blk.add_lwpolyline([(p.x, p.y) for p in pts],
                                   dxfattribs={"layer": ell.dxf.layer, "closed": ell.closed})
    ell.destroy()
    return lw

def spline_to_lwpolyline(msp_or_blk, sp: Spline, approx_seg=1.0):
    try:
        segs = max(16, int(max(sp.length(), 1.0) / max(approx_seg, 0.1)))
        pts = sp.approximate(segments=segs)
    except Exception:
        pts = sp.approximate(segments=128)
    lw = msp_or_blk.add_lwpolyline(pts, dxfattribs={"layer": sp.dxf.layer, "closed": False})
    sp.destroy()
    return lw

def convert_curves(layout):
    for sp in list(layout.query("SPLINE")):
        spline_to_lwpolyline(layout, sp)
    for el in list(layout.query("ELLIPSE")):
        ellipse_to_lwpolyline(layout, el)

# ---------- POLYLINE HELPERS ----------

def lwpolyline_points_xyb(lw) -> Tuple[List[Tuple[float, float, float]], bool]:
    pts = []
    for x, y, *rest in lw.get_points("xyb"):
        bulge = rest[0] if rest else 0.0
        pts.append((float(x), float(y), float(bulge)))
    return pts, bool(lw.closed)

def set_lwpolyline_points(lw, pts: List[Tuple[float, float, float]], closed: bool):
    lw.set_points(pts, format="xyb")
    lw.closed = closed

def poly_total_length_xyb(pts: List[Tuple[float, float, float]], closed: bool) -> float:
    n = len(pts)
    if n < 2:
        return 0.0
    total = 0.0
    last = n if closed else n - 1
    for i in range(last):
        j = (i + 1) % n
        x0, y0, _ = pts[i]
        x1, y1, _ = pts[j]
        total += hypot(x1 - x0, y1 - y0)
    return total

def segment_metrics_xyb(pts: List[Tuple[float, float, float]], closed: bool):
    n = len(pts)
    if n < 2:
        return []
    segs = []
    last = n if closed else n - 1

    def chord(i, j):
        x0, y0, _ = pts[i]
        x1, y1, _ = pts[j]
        return hypot(x1 - x0, y1 - y0), ((x0 + x1) / 2.0, (y0 + y1) / 2.0)

    for i in range(last):
        j = (i + 1) % n
        straight = abs(pts[i][2]) < 1e-12
        ch_len, mid = chord(i, j)
        segs.append({"i0": i, "i1": j, "straight": straight, "chord_len": ch_len, "mid": mid})
    return segs

def choose_segment_for_start(segs: List[dict]) -> Optional[dict]:
    straight = [s for s in segs if s["straight"] and s["chord_len"] >= SEGMENT_MIN_LEN]
    if straight:
        straight.sort(key=lambda s: s["chord_len"], reverse=True)
        return straight[0]
    arcs = [s for s in segs if not s["straight"] and s["chord_len"] >= SEGMENT_MIN_LEN]
    if arcs:
        arcs.sort(key=lambda s: s["chord_len"], reverse=True)
        return arcs[0]
    return None

def insert_midpoint_as_first(pts: List[Tuple[float, float, float]], closed: bool) -> Tuple[List[Tuple[float, float, float]], bool]:
    if len(pts) < 2:
        return pts, closed
    if poly_total_length_xyb(pts, closed) < OPEN_CLOSED_MIN_TOTAL_LEN:
        return pts, closed
    segs = segment_metrics_xyb(pts, closed)
    chosen = choose_segment_for_start(segs)
    if not chosen:
        return pts, closed
    i0 = chosen["i0"]
    midx, midy = chosen["mid"]
    new_pts = []
    for k, (x, y, b) in enumerate(pts):
        new_pts.append((x, y, b))
        if k == i0:
            new_pts.append((midx, midy, 0.0))
    # rotate so the new midpoint becomes the first vertex
    mid_index = None
    for idx in range(len(new_pts) - 1):
        if new_pts[idx][0] == pts[i0][0] and new_pts[idx][1] == pts[i0][1]:
            mid_index = idx + 1
            break
    if mid_index is None:
        for idx, (x, y, b) in enumerate(new_pts):
            if abs(x - midx) < 1e-9 and abs(y - midy) < 1e-9:
                mid_index = idx
                break
    if mid_index is None:
        return pts, closed
    rotated = new_pts[mid_index:] + new_pts[:mid_index]
    return rotated, closed

# ---------- POCKETS / CIRCLES / MILLING ----------

def route_pockets_in_layout(layout, thickness_T: float, material: str):
    """
    Pocket rule: ONLY for CLOSED LWPOLYLINE on 'OCL_PART___<depth>Z' (no suffix).
    Circles are NOT handled here (small holes should be drilling).
    OUTER is skipped here by guard.
    Destination layer is F_Pocket_<depth> regardless of material.
    """
    for e in list(layout):
        try:
            if not hasattr(e.dxf, "layer"):
                continue
            ln = e.dxf.layer or ""
            if OCL_OUTER_RE.match(ln):
                continue  # never touch OUTER here
            m = POCKET_LAYER_RE.match(ln)
            if not m:
                continue
            D = parse_numz(m.group(1))
            if D is None:
                continue
            R = thickness_T - D
            if R <= THROUGH_TOL_MM:
                # through -> handled later by milling rules
                continue
            if e.dxftype() == "LWPOLYLINE":
                try:
                    if bool(e.closed):
                        dest = get_pocket_layer(D)
                        ensure_layer(layout.doc, dest, ACI_PURPLE)
                        e.dxf.layer = dest
                except Exception:
                    pass
        except Exception:
            continue

def process_circles_in_layout(doc: ezdxf.EzfDocument, layout, thickness_T: float, material: str):
    """Drilling/milling for circles with block-aware depth parsing."""
    for c in list(layout.query("CIRCLE")):
        try:
            layer_name = effective_layer_name(c, layout, doc)
            depth = parse_depth_from_layer(layer_name)
            r = float(c.dxf.radius)
            d = 2.0 * r

            if depth is None:
                continue

            if d <= SIX_MM_DIAM:
                # Drilling: normalize to Ø5 (radius 2.5 exactly)
                c.dxf.radius = FIVE_MM_DIAM / 2.0
                if abs(depth - thickness_T) <= THROUGH_TOL_MM:
                    dest = f"V_DrillSF_{fmt_val(thickness_T)}"  # through drill (red)
                    ensure_layer(doc, dest, ACI_RED)
                else:
                    dest = f"V_DrillSF_{fmt_val(depth)}"        # blind drill (turquoise)
                    ensure_layer(doc, dest, ACI_TURQUOISE)
                c.dxf.layer = dest
            else:
                # Milling for large circles
                if abs(depth - thickness_T) <= THROUGH_TOL_MM:
                    dest = get_through_milling_layer(material)
                    ensure_layer(doc, dest, ACI_PURPLE)
                else:
                    dest = get_pocket_layer(depth)
                    ensure_layer(doc, dest, ACI_PURPLE)
                c.dxf.layer = dest

        except Exception:
            continue

def convert_classic_polylines_to_lw(layout):
    for pl in list(layout.query("POLYLINE")):
        try:
            pts = []
            closed = bool(pl.is_closed) if hasattr(pl, "is_closed") else False
            for v in pl.vertices():
                loc = v.dxf.location
                b = float(getattr(v.dxf, "bulge", 0.0) or 0.0)
                pts.append((float(loc.x), float(loc.y), b))
            layout.add_lwpolyline(
                pts, format="xyb",
                dxfattribs={"layer": pl.dxf.layer, "closed": closed}
            )
            try:
                pl.destroy()
            except Exception:
                pass
        except Exception:
            continue

def rename_layers_in_layout(layout, material: str):
    """
    Map OCL_PART___*_OUTER entities to the material-specific outer contour layer.
    (HOLES handled dynamically elsewhere.)
    """
    outer = get_outer_layer(material)
    for e in list(layout):
        try:
            if not hasattr(e.dxf, "layer"):
                continue
            ln = e.dxf.layer or ""
            if OCL_OUTER_RE.match(ln):
                e.dxf.layer = outer
                continue
        except Exception:
            continue

def process_polylines_in_layout(layout):
    for lw in list(layout.query("LWPOLYLINE")):
        try:
            pts, closed = lwpolyline_points_xyb(lw)
            new_pts, new_closed = insert_midpoint_as_first(pts, closed)
            set_lwpolyline_points(lw, new_pts, new_closed)
        except Exception:
            continue

def route_large_holes_polylines(layout, thickness_T: float, material: str):
    """
    Closed polys: mill by depth vs thickness.
    Protect OUTER: skip if already on the material outer layer or source layer matches OUTER.
    Ensure purple for destination layers.
    """
    outer = get_outer_layer(material)
    for lw in list(layout.query("LWPOLYLINE")):
        try:
            ln_current = (lw.dxf.layer or "")
            if ln_current == outer:
                continue  # protected OUTER already mapped

            ln_source = ln_current
            try:
                ln_source = effective_layer_name(lw, layout, lw.doc) if hasattr(lw, "doc") else ln_current
            except Exception:
                pass
            if OCL_OUTER_RE.match(ln_source):
                continue  # don't reroute OUTER contours

            depth = parse_depth_from_layer(ln_source)
            if depth is None:
                continue

            if lw.closed:
                if abs(depth - thickness_T) <= THROUGH_TOL_MM:
                    dest = get_through_milling_layer(material)
                    ensure_layer(layout.doc, dest, ACI_PURPLE)
                else:
                    dest = get_pocket_layer(depth)
                    ensure_layer(layout.doc, dest, ACI_PURPLE)
                lw.dxf.layer = dest
        except Exception:
            continue

def process_layout(doc: ezdxf.EzfDocument, layout, thickness_T: float, material: str):
    """
    Order of operations:
      1) SPLINE/ELLIPSE -> LWPOLYLINE
      2) OUTER mapping (protect OUTER before any routing)
      3) Pocket routing (CLOSED LWPOLYLINE only; circles ignored)
      4) CIRCLE routing (drill/mill) with effective layer parsing
      5) Classic POLYLINE -> LWPOLYLINE
      6) Polyline start-vertex (no break)
      7) Large-hole routing for closed polys (general; skip OUTER)
    """
    convert_curves(layout)
    rename_layers_in_layout(layout, material)                        # OUTER FIRST (protect)
    route_pockets_in_layout(layout, thickness_T, material)           # pockets only for closed LWPOLYLINE
    process_circles_in_layout(doc, layout, thickness_T, material)
    convert_classic_polylines_to_lw(layout)
    process_polylines_in_layout(layout)
    route_large_holes_polylines(layout, thickness_T, material)

# ---------- MATERIAL DETECTION & NAMING ----------

BIN_SINGLE_RE = re.compile(r"^bin_(\d+)$", re.IGNORECASE)
BIN_RANGE_RE  = re.compile(r"^bin_(\d+)_to_(\d+)$", re.IGNORECASE)

RE_WORD_PLYWOOD = re.compile(r"\bPLYWOOD\b", re.IGNORECASE)
RE_TOKEN_PL     = re.compile(r"\b(PL-[A-Za-z0-9._-]+)\b", re.IGNORECASE)
RE_TOKEN_WV     = re.compile(r"\b(WV-[A-Za-z0-9._-]+)\b", re.IGNORECASE)

def _safe_replace_chars(s: str) -> str:
    bad = '<>:"/\\|?*'
    out = "".join("_" if ch in bad else ch for ch in s)
    out = out.strip().rstrip(".")
    return out or "part"

def _norm_mat_token(token: str) -> str:
    t = (token or "").strip().upper().replace(" ", "-")
    return _safe_replace_chars(t)

def _extract_material_from_text(s: str) -> Optional[str]:
    if not s:
        return None
    if RE_WORD_PLYWOOD.search(s):
        return "PLYWOOD"
    m = RE_TOKEN_PL.search(s)
    if m:
        return _norm_mat_token(m.group(1))
    m = RE_TOKEN_WV.search(s)
    if m:
        return _norm_mat_token(m.group(1))
    return None

def detect_material(doc: ezdxf.EzfDocument, in_path: Path) -> str:
    """Heuristics to detect material: filename, then TEXT/MTEXT, then block ATTRIBs."""
    # 1) Filename
    stem = in_path.stem
    mat = _extract_material_from_text(stem)
    if mat:
        return mat

    # 2) TEXT / MTEXT in modelspace
    try:
        msp = doc.modelspace()
        for txt in msp.query("TEXT"):
            try:
                mat = _extract_material_from_text(txt.dxf.text or "")
                if mat:
                    return mat
            except Exception:
                pass
        for mt in msp.query("MTEXT"):
            try:
                mat = _extract_material_from_text(mt.text or "")
                if mat:
                    return mat
            except Exception:
                pass
    except Exception:
        pass

    # 3) Block attributes (INSERT with ATTRIBS)
    try:
        for ins in doc.modelspace().query("INSERT"):
            for att in getattr(ins, "attribs", []):
                try:
                    tag = (att.dxf.tag or "").upper()
                    val = att.dxf.text or ""
                    if tag in {"MATERIAL", "MAT", "MATL"}:
                        mat = _extract_material_from_text(val)
                        if mat:
                            return mat
                    # Or value might contain recognizable tokens
                    mat = _extract_material_from_text(val)
                    if mat:
                        return mat
                except Exception:
                    pass
    except Exception:
        pass

    return "MATERIAL"

def make_output_name_material_sheets_with_mat(in_path: Path, material: str) -> str:
    """
    Convert:
      bin_001.dxf           -> 1-<MATERIAL>-SHEET X 1.dxf
      bin_002_to_005.dxf    -> 2-<MATERIAL>-SHEET X 4.dxf
    Fallback: <stem>-<MATERIAL>.dxf if pattern not matched.
    """
    stem = in_path.stem
    m1 = BIN_SINGLE_RE.match(stem)
    if m1:
        left = int(m1.group(1))
        qty = 1
        return f"{left}-{material}-SHEET X {qty}.dxf"

    m2 = BIN_RANGE_RE.match(stem)
    if m2:
        left = int(m2.group(1))
        right = int(m2.group(2))
        qty = right - left + 1 if right >= left else 1
        return f"{left}-{material}-SHEET X {qty}.dxf"

    clean_stem = _safe_replace_chars(stem)
    return f"{clean_stem}-{material}.dxf"

def _material_from_folder(in_path: Path) -> Optional[str]:
    """
    Extract material from the first subfolder under inbox_raw_dxf.
    Examples:
      'PLYWOOD - 19_05000 mm' -> 'PLYWOOD'
      'PL-1 - 19_05000 mm'   -> 'PL-1'
      'WV-12 - something'    -> 'WV-12'
    """
    try:
        # Find 'inbox_raw_dxf' in the path
        parts = [p for p in in_path.resolve().parts]
        lower_parts = [p.lower() for p in parts]
        if "inbox_raw_dxf" in lower_parts:
            idx = lower_parts.index("inbox_raw_dxf")
            material_folder = parts[idx + 1] if idx + 1 < len(parts) else None
            if material_folder:
                # Split by " - " and take the first token
                return material_folder.split(" - ")[0].strip()
    except Exception:
        pass
    return None

# ---------- NEW: OCL_BIN -> ProcPart_T remap ----------

def remap_bin_to_procpart(doc: ezdxf.EzfDocument, thickness_T: float):
    """
    Move anything on OCL_BIN to ProcPart_{T} where T is the sheet thickness
    rounded with fmt_val (1 decimal, drop .0). Color the ProcPart_* layer blue.
    """
    dest = f"ProcPart_{fmt_val(thickness_T)}"
    ensure_layer(doc, dest, ACI_BLUE)  # ensure layer exists + blue color

    # Modelspace
    for e in list(doc.modelspace()):
        try:
            if getattr(e.dxf, "layer", "") == "OCL_BIN":
                e.dxf.layer = dest
        except Exception:
            pass

    # All blocks (including anonymous)
    for name in doc.blocks.block_names():
        blk = doc.blocks.get(name)
        for e in list(blk):
            try:
                if getattr(e.dxf, "layer", "") == "OCL_BIN":
                    e.dxf.layer = dest
            except Exception:
                pass

# ---------- CLEANUP, COLORS, SAVE ----------

def purge_unused_layers(doc: ezdxf.EzfDocument):
    used = set()
    for e in doc.modelspace():
        if hasattr(e.dxf, "layer"):
            used.add(e.dxf.layer)
    for name in doc.blocks.block_names():  # include anonymous
        blk = doc.blocks.get(name)
        for e in blk:
            if hasattr(e.dxf, "layer"):
                used.add(e.dxf.layer)
    # always keep dynamic prefixes
    keep_prefixes = ("ProcPart_", "V_DrillSF_", "V_Fraes_", "F_Pocket_", "0", "Defpoints")

    for layer in list(doc.layers):
        ln = layer.dxf.name
        if ln.upper().startswith("OCL_"):
            try:
                doc.layers.remove(ln)
            except Exception:
                pass
            continue
        if ln in {"V_DrillSF_14", "V_DrillSF_19", "ProcPart_19"}:
            try:
                doc.layers.remove(ln)
            except Exception:
                pass
            continue
        if ln not in used and not ln.startswith(keep_prefixes):
            try:
                doc.layers.remove(ln)
            except Exception:
                pass

def final_cleanup_layers(doc: ezdxf.EzfDocument, material: str):
    # Ensure any stragglers on OUTER are mapped to the correct outer contour layer
    outer = get_outer_layer(material)
    for layout in doc.layouts:
        for e in list(layout):
            if hasattr(e.dxf, "layer"):
                ln = e.dxf.layer or ""
                if OCL_OUTER_RE.match(ln):
                    e.dxf.layer = outer
    for name in doc.blocks.block_names():
        blk = doc.blocks.get(name)
        for e in list(blk):
            if hasattr(e.dxf, "layer"):
                ln = e.dxf.layer or ""
                if OCL_OUTER_RE.match(ln):
                    e.dxf.layer = outer
    # Current layer safe
    try:
        doc.layers.set_current("0")
    except Exception:
        pass

def force_entities_bylayer_colors(doc: ezdxf.EzfDocument):
    def fix_layout(layout):
        for e in layout:
            # set entity color to BYLAYER (256) even for INSERTs
            if hasattr(e.dxf, "color"):
                try:
                    e.dxf.color = 256
                except Exception:
                    pass
            # clear entity true-color if present
            try:
                if e.dxf.hasattr("true_color"):
                    del e.dxf.true_color
            except Exception:
                pass

    fix_layout(doc.modelspace())

    for name in doc.blocks.block_names():
        blk = doc.blocks.get(name)
        fix_layout(blk)

def apply_layer_palette(doc: ezdxf.EzfDocument, thickness_T: float):
    """
    Normalize ACI colors for all dynamic layers after routing.
    - Clear true_color on layers so ACI is used.
    - V_DrillSF_{T} -> RED
    - All other V_DrillSF_* (blind) -> TURQUOISE
    - V_Fraes_2T134R -> GREEN
    - V_Fraes_*T134L -> PURPLE
    - ProcPart_* -> BLUE
    """
    t_name = f"V_DrillSF_{fmt_val(thickness_T)}"
    for lyr in doc.layers:
        name = lyr.dxf.name or ""
        try:
            if hasattr(lyr.dxf, "true_color"):
                del lyr.dxf.true_color
        except Exception:
            pass

        try:
            if name == t_name:
                lyr.color = ACI_RED
            elif name.startswith("V_DrillSF_"):
                lyr.color = ACI_TURQUOISE
            elif name.startswith("V_Fraes_2T") and name.endswith("R"):
                lyr.color = ACI_GREEN   # outer contour (any tool: 134R, 137R, …)
            elif name.startswith("V_Fraes_"):
                lyr.color = ACI_PURPLE
            elif name.startswith("F_Pocket_"):
                lyr.color = ACI_PURPLE
            elif name.startswith("ProcPart_"):
                lyr.color = ACI_BLUE
        except Exception:
            pass

def paint_entities_from_layer(doc: ezdxf.EzfDocument):
    """
    Force entity display colors to match their layer ACI explicitly.
    This bypasses viewers that ignore ByLayer or apply overrides.
    """
    layer_aci = {}
    for lyr in doc.layers:
        try:
            if hasattr(lyr.dxf, "true_color"):
                del lyr.dxf.true_color
        except Exception:
            pass
        try:
            layer_aci[lyr.dxf.name] = int(lyr.color)
        except Exception:
            layer_aci[lyr.dxf.name] = 256

    def paint_layout(layout):
        for e in layout:
            ln = getattr(e.dxf, "layer", None)
            if not ln:
                continue
            aci = layer_aci.get(ln, 256)
            try:
                if e.dxf.hasattr("true_color"):
                    del e.dxf.true_color
            except Exception:
                pass
            if hasattr(e.dxf, "color"):
                try:
                    e.dxf.color = aci
                except Exception:
                    pass

    paint_layout(doc.modelspace())
    for name in doc.blocks.block_names():
        paint_layout(doc.blocks.get(name))

def safe_save(doc: ezdxf.EzfDocument, out_path: Path) -> bool:
    try:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if out_path.exists():
            try:
                out_path.chmod(0o666)
            except Exception:
                pass
            for _ in range(3):
                try:
                    out_path.unlink()
                    break
                except PermissionError:
                    time.sleep(0.5)
        tmp = out_path.with_suffix(out_path.suffix + ".tmp")
        if tmp.exists():
            try:
                tmp.chmod(0o666)
            except Exception:
                pass
            try:
                tmp.unlink()
            except Exception:
                pass
        try:
            doc.saveas(tmp)
        except Exception as e:
            print(f"[ERROR] Could not write temp file {tmp}: {e}")
            return False
        try:
            tmp.replace(out_path)
        except PermissionError as e:
            print(f"[ERROR] Replace failed (locked?): {out_path} -> {e}")
            return False
        return True
    except Exception as e:
        print(f"[ERROR] Safe save failed for {out_path}: {e}")
        return False

# ---------- FILE PROCESSING ----------

def process_file(in_path: Path, out_dir: Path) -> Optional[Path]:
    try:
        doc = ezdxf.readfile(in_path)
    except Exception as e:
        print(f"[ERROR] Failed to read {in_path}: {e}")
        return None

    auto_units_to_mm(doc)
    thickness_T = get_material_thickness(doc)

    # Detect material early so routing functions can use it
    material = _material_from_folder(in_path) or detect_material(doc, in_path)
    print(f"[INFO] Material: {material}")

    # ---- Arc Healing (before any routing / layer processing) ---------------
    try:
        from arc_healing import heal_arcs_in_doc, print_arc_heal_report
        _heal_report = heal_arcs_in_doc(doc)
        print_arc_heal_report(_heal_report, in_path.name)
    except Exception as _arc_heal_exc:
        print(f"[WARN] Arc healing skipped: {_arc_heal_exc}")
    # -------------------------------------------------------------------------

    # Remap OCL_BIN -> ProcPart_{T} (blue)
    remap_bin_to_procpart(doc, thickness_T)

    # Ensure base layers exist using material-specific names
    ensure_layer(doc, get_outer_layer(material), ACI_GREEN)
    ensure_layer(doc, get_through_milling_layer(material), ACI_PURPLE)

    # Process modelspace + all blocks
    process_layout(doc, doc.modelspace(), thickness_T, material)
    for name in doc.blocks.block_names():
        blk = doc.blocks.get(name)
        process_layout(doc, blk, thickness_T, material)

    # purge & cleanup
    purge_unused_layers(doc)
    final_cleanup_layers(doc, material)

    # Layer colors & entity colors
    apply_layer_palette(doc, thickness_T)
    force_entities_bylayer_colors(doc)
    paint_entities_from_layer(doc)

    # Output naming uses already-detected material
    new_name = make_output_name_material_sheets_with_mat(in_path, material)
    out_path = out_dir / new_name

    # Avoid accidental overwrite
    if out_path.exists():
        base_stem = out_path.stem
        suffix = out_path.suffix
        n = 1
        while True:
            cand = out_path.with_name(f"{base_stem}__{n}{suffix}")
            if not cand.exists():
                out_path = cand
                break
            n += 1

    ok = safe_save(doc, out_path)
    if ok:
        print(f"[OK] {in_path.name} -> {out_path}")
        return out_path
    else:
        print(f"[ERROR] Failed to write {out_path}")
        return None

def is_dxf(p: Path) -> bool:
    return p.suffix.lower() == ".dxf"

def process_path(input_path: Path, out_dir: Path):
    if input_path.is_dir():
        for p in sorted(input_path.rglob("*.dxf")):
            process_file(p, out_dir)
    elif input_path.is_file() and is_dxf(input_path):
        process_file(input_path, out_dir)
    else:
        print(f"[WARN] {input_path} is not a DXF or folder; skipping.")

def main(argv=None):
    import argparse, sys
    ap = argparse.ArgumentParser(description="DXF normalizer for woodWOP pre-processing (v3.12)")
    ap.add_argument("input", help="Path to a DXF file or a folder containing DXFs")
    ap.add_argument("-o", "--out", default="out_dxf", help="Output folder (default: ./out_dxf)")
    args = ap.parse_args(argv or sys.argv[1:])
    process_path(Path(args.input), Path(args.out))

if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv[1:]))
