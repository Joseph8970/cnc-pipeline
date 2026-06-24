"""
Layer name → machining intent interpreter.

WoodWOP-normalised DXF layers follow these conventions (set by dxf_normalizer.py):

    ProcPart_<T>          workpiece boundary;  T = thickness in mm
    V_DrillSF_<D>         vertical drill;      D = depth (D ≈ T → through)

    Outer contour (material-dependent tool number):
        V_Fraes_2T134R    non-plywood outer contour  (depth=2, right-comp)
        V_Fraes_2T137R    plywood outer contour       (depth=2, right-comp)

    Through-milling (material-dependent tool number):
        V_Fraes_0T132L    non-plywood through-mill
        V_Fraes_0T137L    plywood through-mill

    Pocket / groove routing (any tool, depth from layer name):
        V_Fraes_<D>T<N>L  left-comp  pocket;  D = depth; D=0 → through
        V_Fraes_<D>T<N>R  right-comp routing (non-outer variants)

    Pocket layers (depth encoded in name):
        F_Pocket_<D>      pocket at depth D mm (e.g. F_Pocket_5, F_Pocket_12.5)

    OCL_TEXT              label text – ignored for machining
    OCL_PART              insert reference layer – ignored here
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class LayerType(Enum):
    WORKPIECE      = auto()   # ProcPart_T
    OUTER_CONTOUR  = auto()   # V_Fraes_2T134R (always the outer perimeter)
    VERTICAL_DRILL = auto()   # V_DrillSF_D
    POCKET_LEFT    = auto()   # V_Fraes_DT134L (pocket, dado, groove – left comp)
    POCKET_RIGHT   = auto()   # V_Fraes_DT134R (non-outer right-comp routing)
    TEXT           = auto()   # OCL_TEXT – informational only
    IGNORED        = auto()   # anything else


@dataclass(frozen=True)
class LayerInfo:
    layer_name:   str
    layer_type:   LayerType
    depth_mm:     float        # 0.0 when type is WORKPIECE/TEXT/IGNORED
    thickness_mm: float = 0.0  # populated for WORKPIECE layers only
    tool_number:  str   = ""   # tool number extracted from layer name, e.g. "137" from V_Fraes_2T137R

    @property
    def is_through(self) -> bool:
        """True when the depth equals (or closely matches) a full through cut."""
        return self.depth_mm == 0.0 and self.layer_type in (
            LayerType.POCKET_LEFT, LayerType.POCKET_RIGHT
        )

    @property
    def is_machining(self) -> bool:
        return self.layer_type not in (LayerType.TEXT, LayerType.IGNORED)


# ---------------------------------------------------------------------------
# Compiled patterns
# ---------------------------------------------------------------------------
_NUM_RE     = r"(\d+(?:[._]\d+)?)"        # integer or decimal (dot or underscore)
_PROC_RE    = re.compile(r"^ProcPart_" + _NUM_RE + r"$", re.IGNORECASE)
_DRILL_RE   = re.compile(r"^V_DrillSF_" + _NUM_RE + r"$", re.IGNORECASE)
_OUTER_RE   = re.compile(r"^V_Fraes_2T\d+R$", re.IGNORECASE)
_FRAE_RE    = re.compile(r"^V_Fraes_" + _NUM_RE + r"(T\d+[LR])$", re.IGNORECASE)
_FPOCKET_RE = re.compile(r"^F_Pocket_" + _NUM_RE + r"$", re.IGNORECASE)


def _parse_float(token: str) -> float:
    """Convert layer number tokens to float (handles underscore decimal: 19_1 → 19.1)."""
    return float(token.replace("_", "."))


def interpret_layer(layer_name: str) -> LayerInfo:
    """Parse a DXF layer name and return a LayerInfo descriptor."""

    # ---- workpiece --------------------------------------------------------
    m = _PROC_RE.match(layer_name)
    if m:
        t = _parse_float(m.group(1))
        return LayerInfo(layer_name, LayerType.WORKPIECE, depth_mm=0.0, thickness_mm=t)

    # ---- outer contour (special-cased before generic routing) -------------
    if _OUTER_RE.match(layer_name):
        m2  = re.search(r"T(\d+)R$", layer_name, re.IGNORECASE)
        tno = m2.group(1) if m2 else ""
        return LayerInfo(layer_name, LayerType.OUTER_CONTOUR, depth_mm=2.0, tool_number=tno)

    # ---- vertical drill ---------------------------------------------------
    m = _DRILL_RE.match(layer_name)
    if m:
        d = _parse_float(m.group(1))
        return LayerInfo(layer_name, LayerType.VERTICAL_DRILL, depth_mm=d)

    # ---- generic routing (pockets / grooves) ------------------------------
    m = _FRAE_RE.match(layer_name)
    if m:
        d      = _parse_float(m.group(1))
        suffix = m.group(2).upper()
        ltype  = LayerType.POCKET_LEFT if suffix.endswith("L") else LayerType.POCKET_RIGHT
        m3  = re.search(r"T(\d+)[LR]$", suffix, re.IGNORECASE)
        tno = m3.group(1) if m3 else ""
        return LayerInfo(layer_name, ltype, depth_mm=d, tool_number=tno)

    # ---- F_Pocket_<depth> layers ------------------------------------------
    m = _FPOCKET_RE.match(layer_name)
    if m:
        d = _parse_float(m.group(1))
        return LayerInfo(layer_name, LayerType.POCKET_LEFT, depth_mm=d)

    # ---- informational / ignored ------------------------------------------
    if layer_name.upper().startswith("OCL_TEXT"):
        return LayerInfo(layer_name, LayerType.TEXT, depth_mm=0.0)

    return LayerInfo(layer_name, LayerType.IGNORED, depth_mm=0.0)


def interpret_layers(layer_names: list[str]) -> dict[str, LayerInfo]:
    """Batch-interpret a collection of layer names."""
    return {name: interpret_layer(name) for name in layer_names}
