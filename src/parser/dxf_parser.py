"""
DXF parser — sheet-level extraction.

Reads a normalised DXF file (output of dxf_normalizer.py) and returns a
single ``RawSheet`` containing every machining entity already transformed
to world (sheet) coordinates.

Coordinate transform per INSERT:
    wx = ox + sx*(bx*cos(rot) - by*sin(rot))
    wy = oy + sy*(bx*sin(rot) + by*cos(rot))

where (ox, oy) is the INSERT insert point, rot is the INSERT rotation in
radians, sx/sy are xscale/yscale (usually 1.0), and (bx, by) is a point
in block-local coordinates.

The sheet origin (0, 0) is at the lower-left corner of the ProcPart_<T>
polyline, which is already WoodWOP-compatible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Tuple

import ezdxf
from ezdxf.document import Drawing

from src.parser.layer_interpreter import interpret_layer, LayerType


# ---------------------------------------------------------------------------
# Raw geometry containers  (world coordinates)
# ---------------------------------------------------------------------------

@dataclass
class RawPolyline:
    """LWPolyline as a list of (x, y, bulge) tuples in world coords."""
    layer:    str
    closed:   bool
    vertices: List[Tuple[float, float, float]]  # (x, y, bulge)


@dataclass
class RawCircle:
    """CIRCLE entity in world coords."""
    layer:  str
    cx:     float
    cy:     float
    radius: float


@dataclass
class RawSheet:
    """
    All machining geometry for one DXF sheet, in world coordinates.

    sheet_width / sheet_height  – from the ProcPart_<T> boundary polyline
    thickness_mm                – panel thickness from the same layer name
    source_file                 – DXF filename for traceability
    polylines / circles         – flattened, world-coord entities
    """
    sheet_width:  float
    sheet_height: float
    thickness_mm: float
    source_file:  str
    polylines:    List[RawPolyline] = field(default_factory=list)
    circles:      List[RawCircle]   = field(default_factory=list)


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

class DXFParser:
    """Stateless sheet-level parser."""

    def parse_file(self, dxf_path: Path) -> RawSheet:
        """
        Load *dxf_path* and return one RawSheet with all geometry in
        world coordinates.
        """
        doc: Drawing = ezdxf.readfile(str(dxf_path))
        msp = doc.modelspace()

        sheet_width, sheet_height, thickness_mm = self._sheet_dims(msp)

        polylines: List[RawPolyline] = []
        circles:   List[RawCircle]   = []

        for ins in msp.query("INSERT"):
            bname = ins.dxf.name
            if not bname.upper().startswith("OCL_PART"):
                continue

            blk = doc.blocks.get(bname)
            if blk is None:
                continue

            transform, flip_bulge = self._insert_transform(ins)
            self._extract_block(blk, transform, flip_bulge, polylines, circles)

        return RawSheet(
            sheet_width=sheet_width,
            sheet_height=sheet_height,
            thickness_mm=thickness_mm,
            source_file=dxf_path.name,
            polylines=polylines,
            circles=circles,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _sheet_dims(self, msp) -> Tuple[float, float, float]:
        """Return (width, height, thickness) from the ProcPart_<T> polyline."""
        for ent in msp:
            info = interpret_layer(ent.dxf.layer)
            if info.layer_type != LayerType.WORKPIECE:
                continue
            if type(ent).__name__ != "LWPolyline":
                continue
            pts = list(ent.get_points("xy"))
            if not pts:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            return (
                round(max(xs) - min(xs), 4),
                round(max(ys) - min(ys), 4),
                info.thickness_mm,
            )
        return (2438.4, 1219.2, 18.0)  # standard 4×8 sheet fallback

    @staticmethod
    def _insert_transform(
        ins,
    ) -> Tuple[Callable[[float, float], Tuple[float, float]], bool]:
        """Build a world-coordinate transform from an INSERT entity.

        Returns (transform_fn, flip_bulge).
        flip_bulge is True when mirroring (xscale*yscale < 0) which reverses
        arc direction and therefore flips the bulge sign.
        """
        ox    = float(ins.dxf.insert.x)
        oy    = float(ins.dxf.insert.y)
        rot   = math.radians(float(getattr(ins.dxf, "rotation", 0.0) or 0.0))
        sx    = float(getattr(ins.dxf, "xscale",   1.0) or 1.0)
        sy    = float(getattr(ins.dxf, "yscale",   1.0) or 1.0)
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)

        def transform(bx: float, by: float) -> Tuple[float, float]:
            wx = ox + sx * (bx * cos_r - by * sin_r)
            wy = oy + sy * (bx * sin_r + by * cos_r)
            return wx, wy

        return transform, (sx * sy) < 0

    def _extract_block(
        self,
        blk,
        transform: Callable[[float, float], Tuple[float, float]],
        flip_bulge: bool,
        polylines:  List[RawPolyline],
        circles:    List[RawCircle],
    ) -> None:
        """
        Walk all entities in *blk*, apply the INSERT transform to each vertex,
        and append to the output lists.  Text and ignored layers are skipped.
        """
        for ent in blk:
            info = interpret_layer(ent.dxf.layer)
            if not info.is_machining:
                continue

            etype = type(ent).__name__

            if etype == "LWPolyline":
                verts = self._read_lwpolyline(ent, transform, flip_bulge)
                if verts:
                    polylines.append(RawPolyline(
                        layer=ent.dxf.layer,
                        closed=ent.closed,
                        vertices=verts,
                    ))

            elif etype == "Polyline":
                verts = self._read_polyline(ent, transform, flip_bulge)
                if verts:
                    polylines.append(RawPolyline(
                        layer=ent.dxf.layer,
                        closed=ent.is_closed,
                        vertices=verts,
                    ))

            elif etype == "Circle":
                cx, cy = transform(float(ent.dxf.center.x), float(ent.dxf.center.y))
                circles.append(RawCircle(
                    layer=ent.dxf.layer,
                    cx=cx,
                    cy=cy,
                    radius=float(ent.dxf.radius),
                ))

            elif etype == "Arc":
                verts = self._arc_to_vertices(ent, transform, flip_bulge)
                if verts:
                    polylines.append(RawPolyline(
                        layer=ent.dxf.layer,
                        closed=False,
                        vertices=verts,
                    ))

    # ------------------------------------------------------------------
    # Vertex readers (all accept a transform callable and flip_bulge flag)
    # ------------------------------------------------------------------

    @staticmethod
    def _read_lwpolyline(
        ent,
        transform:  Callable[[float, float], Tuple[float, float]],
        flip_bulge: bool,
    ) -> List[Tuple[float, float, float]]:
        result: List[Tuple[float, float, float]] = []
        try:
            for pt in ent.get_points("xyseb"):
                x, y, _s, _e, bulge = pt
                wx, wy = transform(float(x), float(y))
                b = -float(bulge) if flip_bulge else float(bulge)
                result.append((wx, wy, b))
        except Exception:
            pass
        return result

    @staticmethod
    def _read_polyline(
        ent,
        transform:  Callable[[float, float], Tuple[float, float]],
        flip_bulge: bool,
    ) -> List[Tuple[float, float, float]]:
        result: List[Tuple[float, float, float]] = []
        try:
            for v in ent.vertices:
                wx, wy = transform(float(v.dxf.location.x), float(v.dxf.location.y))
                bulge  = float(v.dxf.bulge) if hasattr(v.dxf, "bulge") else 0.0
                b = -bulge if flip_bulge else bulge
                result.append((wx, wy, b))
        except Exception:
            pass
        return result

    @staticmethod
    def _arc_to_vertices(
        ent,
        transform:  Callable[[float, float], Tuple[float, float]],
        flip_bulge: bool,
    ) -> List[Tuple[float, float, float]]:
        try:
            cx      = float(ent.dxf.center.x)
            cy      = float(ent.dxf.center.y)
            r       = float(ent.dxf.radius)
            a_start = math.radians(float(ent.dxf.start_angle))
            a_end   = math.radians(float(ent.dxf.end_angle))
            span    = a_end - a_start
            if span < 0:
                span += 2 * math.pi
            bulge = math.tan(span / 4.0)
            if flip_bulge:
                bulge = -bulge
            x1, y1 = transform(cx + r * math.cos(a_start), cy + r * math.sin(a_start))
            x2, y2 = transform(cx + r * math.cos(a_end),   cy + r * math.sin(a_end))
            return [(x1, y1, bulge), (x2, y2, 0.0)]
        except Exception:
            return []
