"""
Machining operation dataclasses.

Each class represents one self-contained CNC operation with all geometry
and parameters needed to generate the corresponding MPR block.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from src.geometry.geometry_engine import Segment, Point2D


# ---------------------------------------------------------------------------
# Workpiece specification
# ---------------------------------------------------------------------------

@dataclass
class WorkpieceSpec:
    width_x:      float
    width_y:      float
    thickness_z:  float
    label:        str     = ""
    block_name:   str     = ""
    source_file:  str     = ""


# ---------------------------------------------------------------------------
# Vertical drilling
# ---------------------------------------------------------------------------

@dataclass
class VerticalDrillOperation:
    x:          float
    y:          float
    diameter:   float
    depth:      float
    face:       int   = 1
    tool_number: str  = "60"

    @property
    def is_through(self) -> bool:
        return False


# ---------------------------------------------------------------------------
# Horizontal drilling
# ---------------------------------------------------------------------------

@dataclass
class HorizontalDrillOperation:
    x:          float
    y:          float
    z:          float
    diameter:   float
    depth:      float
    face:       int   = 5
    tool_number: str  = "50"


# ---------------------------------------------------------------------------
# Contour (routing)
# ---------------------------------------------------------------------------

@dataclass
class ContourOperation:
    """
    A closed routing contour.

    through_pass_tool_number: if non-empty, the MPR writer emits a second
    <105> block on the same contour with depth=0 and this tool number.
    This models the two-pass outer-contour strategy: scoring pass first
    (depth from layer, e.g. 2 mm), then through cut (ZA=0).
    """
    segments:                List[Segment]
    depth_mm:                float
    rk:                      str   = "WRKR"
    contour_id:              int   = 1
    tool_number:             str   = "101"
    feed_rate:               float = 10.0
    workstations:            str   = "1,2,3,401,402,403"
    through_pass_tool_number: str   = ""   # non-empty → generate through pass
    through_pass_depth:       float = 0.0  # ZA for the through pass

    @property
    def is_outer(self) -> bool:
        return self.rk == "WRKR"

    @property
    def start_point(self) -> Point2D:
        return self.segments[0].start if self.segments else (0.0, 0.0)

    @property
    def point_count(self) -> int:
        return 1 + len(self.segments)


# ---------------------------------------------------------------------------
# Pocket (closed area removal)
# ---------------------------------------------------------------------------

@dataclass
class PocketOperation:
    segments:   List[Segment]
    depth_mm:   float
    contour_id: int  = 1
    tool_number: str = "101"
    feed_rate:  float = 10.0
    workstations: str = "1,2,3,401,402,403"

    def to_contour_operation(self) -> ContourOperation:
        return ContourOperation(
            segments=self.segments,
            depth_mm=self.depth_mm,
            rk="WRKL",
            contour_id=self.contour_id,
            tool_number=self.tool_number,
            feed_rate=self.feed_rate,
            workstations=self.workstations,
        )


# ---------------------------------------------------------------------------
# Groove (straight dado / slot)
# ---------------------------------------------------------------------------

@dataclass
class GrooveOperation:
    x_start:    float
    y_start:    float
    x_end:      float
    y_end:      float
    depth_mm:   float
    width_mm:   float
    tool_number: str = "40"
    workstations: str = "40,41,42,45,141,142"


# ---------------------------------------------------------------------------
# Container for all operations belonging to one part
# ---------------------------------------------------------------------------

@dataclass
class PartOperations:
    workpiece:       WorkpieceSpec
    vertical_drills: List[VerticalDrillOperation]  = field(default_factory=list)
    horizontal_drills: List[HorizontalDrillOperation] = field(default_factory=list)
    contours:        List[ContourOperation]        = field(default_factory=list)
    pockets:         List[PocketOperation]         = field(default_factory=list)
    grooves:         List[GrooveOperation]         = field(default_factory=list)

    @property
    def has_any_machining(self) -> bool:
        return bool(
            self.vertical_drills
            or self.horizontal_drills
            or self.contours
            or self.pockets
            or self.grooves
        )

    @property
    def operation_count(self) -> int:
        return (
            len(self.vertical_drills)
            + len(self.horizontal_drills)
            + len(self.contours)
            + len(self.pockets)
            + len(self.grooves)
        )
