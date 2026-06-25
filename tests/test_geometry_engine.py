"""Tests for the geometry engine."""

import math
import pytest

from src.geometry.geometry_engine import (
    ArcSegment,
    BBox,
    CoordinateTransform,
    LineSegment,
    bbox_of_points,
    close_contour,
    contour_is_closed,
    is_ccw,
    lwpolyline_to_segments,
    merge_consecutive_arcs,
    signed_area,
)


class TestLineSegment:
    def test_departure_angle_right(self):
        seg = LineSegment((0, 0), (10, 0))
        assert abs(seg.departure_angle - 0.0) < 1e-9

    def test_departure_angle_up(self):
        seg = LineSegment((0, 0), (0, 10))
        assert abs(seg.departure_angle - math.pi / 2) < 1e-9

    def test_departure_angle_left(self):
        seg = LineSegment((10, 0), (0, 0))
        assert abs(seg.departure_angle - math.pi) < 1e-9

    def test_departure_angle_down(self):
        seg = LineSegment((0, 10), (0, 0))
        assert abs(seg.departure_angle - 3 * math.pi / 2) < 1e-9

    def test_length(self):
        seg = LineSegment((0, 0), (3, 4))
        assert abs(seg.length - 5.0) < 1e-9

    def test_translated(self):
        seg = LineSegment((1, 2), (3, 4))
        t   = seg.translated(10, 20)
        assert t.start == (11, 22)
        assert t.end   == (13, 24)


class TestSignedArea:
    def test_ccw_square(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        assert signed_area(pts) > 0.0
        assert is_ccw(pts)

    def test_cw_square(self):
        pts = [(0, 0), (0, 1), (1, 1), (1, 0)]
        assert signed_area(pts) < 0.0
        assert not is_ccw(pts)

    def test_rectangle_area(self):
        pts = [(0, 0), (10, 0), (10, 5), (0, 5)]
        assert abs(abs(signed_area(pts)) - 50.0) < 1e-6


class TestBBox:
    def test_basic(self):
        bb = bbox_of_points([(1, 2), (5, 3), (3, 7)])
        assert bb.min_x == 1
        assert bb.min_y == 2
        assert bb.max_x == 5
        assert bb.max_y == 7
        assert bb.width  == 4
        assert bb.height == 5


class TestCoordinateTransform:
    def test_offset_from_outer_contour(self):
        # Outer contour centred at origin: x ∈ [-100, 100], y ∈ [-50, 50]
        verts = [(-100, -50), (100, -50), (100, 50), (-100, 50)]
        t = CoordinateTransform.from_outer_contour(verts)
        assert t.offset_x == 100.0
        assert t.offset_y == 50.0

    def test_apply(self):
        t  = CoordinateTransform(offset_x=100.0, offset_y=50.0)
        wx, wy = t.apply(-100, -50)
        assert abs(wx - 0.0) < 1e-9
        assert abs(wy - 0.0) < 1e-9

    def test_apply_top_right_corner(self):
        t  = CoordinateTransform(offset_x=100.0, offset_y=50.0)
        wx, wy = t.apply(100, 50)
        assert abs(wx - 200.0) < 1e-9
        assert abs(wy - 100.0) < 1e-9


class TestLWPolylineToSegments:
    def test_rectangle_no_bulge(self):
        # 4-vertex rectangle (closed)
        verts = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0),
                 (10.0, 5.0, 0.0), (0.0, 5.0, 0.0)]
        segs = lwpolyline_to_segments(verts, closed=True)
        assert len(segs) == 4
        assert all(isinstance(s, LineSegment) for s in segs)

    def test_open_polyline(self):
        verts = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (5.0, 3.0, 0.0)]
        segs = lwpolyline_to_segments(verts, closed=False)
        assert len(segs) == 2

    def test_bulge_creates_arc(self):
        # Semicircle: two points with bulge = tan(90°/2) = 1.0
        verts = [(0.0, 0.0, 1.0), (2.0, 0.0, 0.0)]
        segs = lwpolyline_to_segments(verts, closed=False)
        assert len(segs) == 1
        assert isinstance(segs[0], ArcSegment)


class TestMergeConsecutiveArcs:
    def test_two_quarter_circles_merge(self):
        # Two consecutive CCW quarter-arcs on the unit circle → one half-arc
        # bulge for 90° arc = tan(π/8) ≈ 0.41421
        b = math.tan(math.pi / 8)
        verts = [(1.0, 0.0, b), (0.0, 1.0, b), (-1.0, 0.0, 0.0)]
        segs = lwpolyline_to_segments(verts, closed=False)
        assert len(segs) == 1
        assert isinstance(segs[0], ArcSegment)
        assert segs[0].start == (1.0, 0.0)
        assert abs(segs[0].end[0] - (-1.0)) < 0.001
        assert segs[0].ccw

    def test_arcs_on_different_circles_not_merged(self):
        # Two arcs with clearly different centers should stay separate
        arc1 = ArcSegment(start=(0.0, 0.0), end=(1.0, 0.0),
                          center=(0.5, 0.5), radius=0.707, ccw=True)
        arc2 = ArcSegment(start=(1.0, 0.0), end=(2.0, 0.0),
                          center=(1.5, -5.0), radius=5.1, ccw=True)
        result = merge_consecutive_arcs([arc1, arc2])
        assert len(result) == 2

    def test_line_between_arcs_prevents_merge(self):
        b = math.tan(math.pi / 8)
        verts = [(1.0, 0.0, b), (0.0, 1.0, 0.0), (-1.0, 0.0, b), (0.0, -1.0, 0.0)]
        segs = lwpolyline_to_segments(verts, closed=False)
        # arc, line, arc — the two arcs must not merge
        assert len(segs) == 3
        assert isinstance(segs[0], ArcSegment)
        assert isinstance(segs[1], LineSegment)
        assert isinstance(segs[2], ArcSegment)

    def test_opposite_direction_arcs_not_merged(self):
        arc1 = ArcSegment(start=(0.0, 0.0), end=(1.0, 0.0),
                          center=(0.5, 0.0), radius=0.5, ccw=True)
        arc2 = ArcSegment(start=(1.0, 0.0), end=(2.0, 0.0),
                          center=(1.5, 0.0), radius=0.5, ccw=False)
        result = merge_consecutive_arcs([arc1, arc2])
        assert len(result) == 2


class TestContourClosure:
    def test_closed(self):
        segs = [
            LineSegment((0, 0), (10, 0)),
            LineSegment((10, 0), (10, 5)),
            LineSegment((10, 5), (0, 5)),
            LineSegment((0, 5), (0, 0)),
        ]
        assert contour_is_closed(segs)

    def test_open_and_close(self):
        segs = [
            LineSegment((0, 0), (10, 0)),
            LineSegment((10, 0), (10, 5)),
        ]
        assert not contour_is_closed(segs)
        closed = close_contour(segs)
        assert contour_is_closed(closed)
        assert len(closed) == 3
