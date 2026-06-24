"""
Geometry engine.

Responsibilities:
  - Represent DXF geometry as typed segments (LineSegment / ArcSegment).
  - Convert LWPolyline vertices (with bulge) into segments.
  - Compute bounding boxes and winding direction (CW / CCW).
  - Transform block-local coordinates to WoodWOP part coordinates
    (origin at bottom-left corner of the outer contour).
  - Compute tangent-departure angles (radians) for each segment end-point,
    as required by the WoodWOP MPR contour section.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple


# ---------------------------------------------------------------------------
# Coordinate type alias
# ---------------------------------------------------------------------------
Point2D = Tuple[float, float]


# ---------------------------------------------------------------------------
# Segment types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LineSegment:
    start: Point2D
    end:   Point2D

    @property
    def length(self) -> float:
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        return math.hypot(dx, dy)

    @property
    def departure_angle(self) -> float:
        """Angle of travel from start→end, in radians [0, 2π)."""
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        a  = math.atan2(dy, dx)
        return a % (2 * math.pi)

    def translated(self, dx: float, dy: float) -> "LineSegment":
        return LineSegment(
            (self.start[0] + dx, self.start[1] + dy),
            (self.end[0]   + dx, self.end[1]   + dy),
        )


@dataclass(frozen=True)
class ArcSegment:
    """
    A circular arc from ``start`` to ``end`` passing through the arc defined
    by ``center`` and ``radius``.

    ``ccw=True`` means the arc is traversed counter-clockwise.
    """
    start:  Point2D
    end:    Point2D
    center: Point2D
    radius: float
    ccw:    bool = True

    @property
    def departure_angle(self) -> float:
        """Tangent direction at the *end* point, in radians [0, 2π)."""
        cx, cy = self.center
        ex, ey = self.end
        # Vector from center to end point
        radial_angle = math.atan2(ey - cy, ex - cx)
        # Tangent is perpendicular to radial
        if self.ccw:
            tangent = radial_angle + math.pi / 2
        else:
            tangent = radial_angle - math.pi / 2
        return tangent % (2 * math.pi)

    def translated(self, dx: float, dy: float) -> "ArcSegment":
        return ArcSegment(
            (self.start[0]  + dx, self.start[1]  + dy),
            (self.end[0]    + dx, self.end[1]    + dy),
            (self.center[0] + dx, self.center[1] + dy),
            self.radius,
            self.ccw,
        )


Segment = LineSegment | ArcSegment


# ---------------------------------------------------------------------------
# Bounding box
# ---------------------------------------------------------------------------

@dataclass
class BBox:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_y - self.min_y

    @property
    def center(self) -> Point2D:
        return (self.min_x + self.width / 2, self.min_y + self.height / 2)


def bbox_of_points(points: Sequence[Point2D]) -> BBox:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return BBox(min(xs), min(ys), max(xs), max(ys))


# ---------------------------------------------------------------------------
# Bulge → arc conversion
# ---------------------------------------------------------------------------

def _arc_from_bulge(
    p1: Point2D,
    p2: Point2D,
    bulge: float,
) -> ArcSegment:
    """
    Convert an LWPOLYLINE bulge value between two vertices to an ArcSegment.

    Bulge = tan(included_angle / 4).  Positive bulge → CCW arc.
    """
    x1, y1 = p1
    x2, y2 = p2
    ccw = bulge > 0

    # Half-angle and sagitta
    theta  = 4.0 * math.atan(abs(bulge))        # included arc angle
    d      = math.hypot(x2 - x1, y2 - y1)       # chord length
    r      = d / (2.0 * math.sin(theta / 2.0))  # radius

    # Midpoint of chord
    mx = (x1 + x2) / 2.0
    my = (y1 + y2) / 2.0

    # Perpendicular to chord
    chord_angle = math.atan2(y2 - y1, x2 - x1)
    perp_angle  = chord_angle + math.pi / 2.0

    # Distance from chord midpoint to center
    dist = math.sqrt(max(r * r - (d / 2.0) ** 2, 0.0))

    # For CCW arc (positive bulge), center is to the left of chord direction
    if ccw:
        cx = mx - dist * math.cos(perp_angle)
        cy = my - dist * math.sin(perp_angle)
    else:
        cx = mx + dist * math.cos(perp_angle)
        cy = my + dist * math.sin(perp_angle)

    return ArcSegment(p1, p2, (cx, cy), r, ccw)


# ---------------------------------------------------------------------------
# LWPolyline → segments
# ---------------------------------------------------------------------------

def lwpolyline_to_segments(
    vertices: List[Tuple[float, float, float]],  # (x, y, bulge)
    closed: bool,
) -> List[Segment]:
    """
    Convert a list of (x, y, bulge) tuples from an LWPOLYLINE entity into
    a list of LineSegment / ArcSegment objects.

    For a *closed* polyline the last vertex connects back to the first.
    """
    segments: List[Segment] = []
    n = len(vertices)
    if n < 2:
        return segments

    limit = n if closed else n - 1
    for i in range(limit):
        x1, y1, b1 = vertices[i]
        x2, y2, _  = vertices[(i + 1) % n]
        p1: Point2D = (x1, y1)
        p2: Point2D = (x2, y2)

        if abs(b1) < 1e-9:
            seg: Segment = LineSegment(p1, p2)
        else:
            seg = _arc_from_bulge(p1, p2, b1)
        segments.append(seg)

    return segments


def extract_lwpolyline_xybulge(entity) -> List[Tuple[float, float, float]]:
    """
    Pull (x, y, bulge) from an ezdxf LWPolyline entity.
    Uses the 'xyseb' format to capture bulge.
    """
    result: List[Tuple[float, float, float]] = []
    for pt in entity.get_points("xyseb"):
        x, y, _s, _e, bulge = pt
        result.append((float(x), float(y), float(bulge)))
    return result


# ---------------------------------------------------------------------------
# Winding direction
# ---------------------------------------------------------------------------

def signed_area(points: Sequence[Point2D]) -> float:
    """Shoelace formula.  Positive → CCW, negative → CW."""
    n   = len(points)
    acc = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        acc += x1 * y2 - x2 * y1
    return acc / 2.0


def is_ccw(points: Sequence[Point2D]) -> bool:
    return signed_area(points) > 0.0


# ---------------------------------------------------------------------------
# Coordinate transform: block-local → WoodWOP part-local
# ---------------------------------------------------------------------------

@dataclass
class CoordinateTransform:
    """
    Translates from block-local coordinates (origin at part centre as set by
    dxf_normalizer) to WoodWOP part coordinates (origin at bottom-left).
    """
    offset_x: float  # amount to ADD to block_x to get WoodWOP_x
    offset_y: float  # amount to ADD to block_y to get WoodWOP_y

    @classmethod
    def from_outer_contour(cls, outer_contour_vertices: Sequence[Point2D]) -> "CoordinateTransform":
        """
        Derive the transform by finding the minimum-x and minimum-y extent
        of the outer contour.
        """
        xs = [p[0] for p in outer_contour_vertices]
        ys = [p[1] for p in outer_contour_vertices]
        return cls(offset_x=-min(xs), offset_y=-min(ys))

    def apply(self, x: float, y: float) -> Point2D:
        return (x + self.offset_x, y + self.offset_y)

    def apply_point(self, p: Point2D) -> Point2D:
        return self.apply(p[0], p[1])

    def apply_segment(self, seg: Segment) -> Segment:
        dx, dy = self.offset_x, self.offset_y
        if isinstance(seg, LineSegment):
            return seg.translated(dx, dy)
        return seg.translated(dx, dy)

    def apply_segments(self, segments: List[Segment]) -> List[Segment]:
        return [self.apply_segment(s) for s in segments]


# ---------------------------------------------------------------------------
# Contour closing / validation helpers
# ---------------------------------------------------------------------------

def contour_is_closed(segments: List[Segment], tol: float = 0.01) -> bool:
    """Return True if the last segment ends at the first segment's start."""
    if not segments:
        return False
    start = segments[0].start
    end   = segments[-1].end
    return math.hypot(end[0] - start[0], end[1] - start[1]) < tol


def close_contour(segments: List[Segment]) -> List[Segment]:
    """Append a closing LineSegment if the contour is not already closed."""
    if not segments or contour_is_closed(segments):
        return segments
    return segments + [LineSegment(segments[-1].end, segments[0].start)]
