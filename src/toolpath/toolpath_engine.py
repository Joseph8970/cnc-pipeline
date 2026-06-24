"""
Toolpath engine -- sheet-level conversion.

Converts a ``RawSheet`` (world-coordinate DXF geometry) into a
``PartOperations`` object where the workpiece is the full sheet.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

from src.config_manager import Config, get_config
from src.geometry.geometry_engine import (
    BBox,
    LineSegment,
    Segment,
    bbox_of_points,
    close_contour,
    lwpolyline_to_segments,
)
from src.parser.dxf_parser import RawPolyline, RawSheet
from src.parser.layer_interpreter import LayerType, interpret_layer
from src.toolpath.operations import (
    ContourOperation,
    PartOperations,
    PocketOperation,
    VerticalDrillOperation,
    WorkpieceSpec,
)

_log = logging.getLogger("cnc.conversion")


class ToolpathEngine:
    """
    Stateless converter.  Inject a Config object or let it fall back to
    the shared singleton via ``get_config()``.
    """

    def __init__(self, config: Optional[Config] = None) -> None:
        self._cfg = config or get_config()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def process(self, sheet: RawSheet) -> PartOperations:
        """Convert one RawSheet to PartOperations."""
        workpiece = WorkpieceSpec(
            width_x=sheet.sheet_width,
            width_y=sheet.sheet_height,
            thickness_z=sheet.thickness_mm,
            label=sheet.source_file,
            block_name="SHEET",
            source_file=sheet.source_file,
        )

        contours: List[ContourOperation]       = []
        pockets:  List[PocketOperation]        = []
        drills:   List[VerticalDrillOperation] = []

        rt = self._cfg.tools.routing_tools
        contour_id = 1

        # --- routing polylines -------------------------------------------
        for poly in sheet.polylines:
            info = interpret_layer(poly.layer)

            if info.layer_type == LayerType.OUTER_CONTOUR:
                segs = self._poly_to_segments(poly)

                # Tool number encoded in layer name (e.g. "137" from V_Fraes_2T137R).
                scoring_tno = info.tool_number or rt.outer_scoring_tno

                # T130 finish-pass: all outer contours get a second pass.
                # Depth: PLYWOOD uses plywood_finish_depth; all others use
                # large/small threshold based on bounding-box shortest edge.
                through_tno = rt.outer_through_tno
                if scoring_tno == "137":
                    through_depth = rt.plywood_finish_depth
                else:
                    w, h     = self._contour_size(segs)
                    min_edge = min(w, h)
                    through_depth = (
                        rt.large_part_finish_depth
                        if min_edge > rt.small_part_threshold_mm
                        else rt.small_part_finish_depth
                    )

                contours.append(ContourOperation(
                    segments=segs,
                    depth_mm=rt.outer_scoring_depth,
                    rk="WRKR",
                    contour_id=contour_id,
                    tool_number=scoring_tno,
                    feed_rate=self._cfg.machining.default_feed_rate,
                    workstations=rt.workstations,
                    through_pass_tool_number=through_tno,
                    through_pass_depth=through_depth,
                ))
                contour_id += 1

            elif info.layer_type in (LayerType.POCKET_LEFT, LayerType.POCKET_RIGHT):
                if not poly.closed:
                    _log.warning("Open routing polyline on %s -- skipped", poly.layer)
                    continue
                segs  = self._poly_to_segments(poly)
                depth = info.depth_mm if info.depth_mm >= 0 else 0.0
                rk    = "WRKR" if info.layer_type == LayerType.POCKET_RIGHT else "WRKL"

                # Tool number from layer name takes precedence over config fallback.
                # F_Pocket_ layers carry no tool number → fall back to pocket_tno.
                if info.tool_number:
                    tno = info.tool_number
                elif rk == "WRKL":
                    tno = rt.pocket_tno
                else:
                    tno = rt.outer_scoring_tno

                if rk == "WRKL" and depth > 0.0:
                    # Blind pocket → <181 \FreiFormTasche\> in the writer
                    pockets.append(PocketOperation(
                        segments=segs,
                        depth_mm=depth,
                        contour_id=contour_id,
                        tool_number=tno,
                        feed_rate=self._cfg.machining.default_feed_rate,
                        workstations=rt.pocket_workstations,
                    ))
                else:
                    # Through-cut (depth==0) or POCKET_RIGHT → <105 \Konturfraesen\>
                    contours.append(ContourOperation(
                        segments=segs,
                        depth_mm=depth,
                        rk=rk,
                        contour_id=contour_id,
                        tool_number=tno,
                        feed_rate=self._cfg.machining.default_feed_rate,
                        workstations=rt.workstations,
                    ))
                contour_id += 1

        # --- vertical drills ---------------------------------------------
        for circle in sheet.circles:
            info = interpret_layer(circle.layer)
            if info.layer_type != LayerType.VERTICAL_DRILL:
                continue

            diameter = round(circle.radius * 2.0, 4)
            depth    = info.depth_mm
            if abs(depth - sheet.thickness_mm) <= self._cfg.machining.through_tolerance_mm:
                depth = sheet.thickness_mm

            drills.append(VerticalDrillOperation(
                x=round(circle.cx, 4),
                y=round(circle.cy, 4),
                diameter=diameter,
                depth=depth,
                tool_number=self._cfg.tool_number_for_diameter(diameter),
            ))

        return PartOperations(
            workpiece=workpiece,
            vertical_drills=drills,
            contours=contours,
            pockets=pockets,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _poly_to_segments(self, poly: RawPolyline) -> List[Segment]:
        segs = lwpolyline_to_segments(poly.vertices, poly.closed)
        return close_contour(segs)

    @staticmethod
    def _is_rectangle(segs: List[Segment]) -> bool:
        """Return True if *segs* form an axis-aligned rectangle (or square).

        Handles contours where the start point falls mid-edge, which splits one
        side into two collinear segments (5 total instead of the canonical 4).
        Uses direction-change counting after merging collinear adjacent segments.
        """
        _AXIS_TOL = 0.5  # mm

        def _dir(seg: LineSegment) -> str:
            dx = seg.end[0] - seg.start[0]
            dy = seg.end[1] - seg.start[1]
            if abs(dx) >= abs(dy):
                return "H+" if dx >= 0 else "H-"
            return "V+" if dy >= 0 else "V-"

        for seg in segs:
            if not isinstance(seg, LineSegment):
                return False
            dx = abs(seg.end[0] - seg.start[0])
            dy = abs(seg.end[1] - seg.start[1])
            if not (dx < _AXIS_TOL or dy < _AXIS_TOL):
                return False

        # Merge consecutive collinear segments; handle closed-loop wrap so that
        # the last segment merges with the first when both run in the same direction.
        dirs = [_dir(s) for s in segs]  # type: ignore[arg-type]
        merged: List[str] = [dirs[0]]
        for d in dirs[1:]:
            if d != merged[-1]:
                merged.append(d)
        if len(merged) > 1 and merged[-1] == merged[0]:
            merged.pop()

        return len(merged) == 4

    @staticmethod
    def _contour_size(segs: List[Segment]) -> tuple[float, float]:
        """Return (width, height) bounding box of the contour segments."""
        pts = [s.start for s in segs] + [segs[-1].end] if segs else []
        if not pts:
            return 0.0, 0.0
        bb = bbox_of_points(pts)
        return bb.width, bb.height
