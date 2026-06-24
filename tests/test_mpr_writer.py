"""Tests for the MPR writer."""

import math
import pytest

from src.geometry.geometry_engine import LineSegment, ArcSegment
from src.toolpath.operations import (
    ContourOperation,
    PartOperations,
    VerticalDrillOperation,
    WorkpieceSpec,
)
from src.writer.mpr_writer import MPRWriter


def _simple_rectangle_contour(w: float, h: float, cid: int = 1) -> ContourOperation:
    segs = [
        LineSegment((0.0, 0.0), (w, 0.0)),
        LineSegment((w, 0.0),   (w, h)),
        LineSegment((w, h),     (0.0, h)),
        LineSegment((0.0, h),   (0.0, 0.0)),
    ]
    return ContourOperation(
        segments=segs,
        depth_mm=20.1,
        rk="WRKR",
        contour_id=cid,
        tool_number="101",
        feed_rate=10.0,
        workstations="1,2,3",
    )


def _make_ops(w: float = 711.2, h: float = 609.6, t: float = 19.1) -> PartOperations:
    wp = WorkpieceSpec(width_x=w, width_y=h, thickness_z=t,
                       label="TEST", block_name="OCL_TEST", source_file="test.dxf")
    ct = _simple_rectangle_contour(w, h, cid=1)
    return PartOperations(workpiece=wp, contours=[ct])


class TestMPRWriter:
    def setup_method(self):
        self.writer = MPRWriter()

    def test_header_contains_bsx_bsy_bsz(self):
        ops = _make_ops(711.2, 609.6, 19.1)
        mpr = self.writer.write(ops)
        assert "_BSX=711.200000" in mpr
        assert "_BSY=609.600000" in mpr
        assert "_BSZ=19.100000"  in mpr

    def test_workpiece_block_present(self):
        ops = _make_ops()
        mpr = self.writer.write(ops)
        assert "<100 \\WerkStck\\" in mpr
        assert 'LA="711.2"' in mpr
        assert 'BR="609.6"' in mpr

    def test_comment_block_present(self):
        ops = _make_ops()
        mpr = self.writer.write(ops)
        assert "<101 \\Kommentar\\" in mpr

    def test_contour_definition_present(self):
        ops = _make_ops()
        mpr = self.writer.write(ops)
        assert "]1" in mpr
        assert "KP" in mpr
        assert "KL" in mpr

    def test_contour_routing_block_present(self):
        ops = _make_ops()
        mpr = self.writer.write(ops)
        assert "<105 \\Konturfraesen\\" in mpr
        assert 'EA="1:0"' in mpr

    def test_file_ends_with_exclamation(self):
        ops = _make_ops()
        mpr = self.writer.write(ops)
        assert mpr.rstrip().endswith("!")

    def test_vertical_drill_block(self):
        wp = WorkpieceSpec(711.2, 609.6, 19.1, "T", "B", "f.dxf")
        dr = VerticalDrillOperation(x=100.0, y=50.0, diameter=5.0, depth=13.0, tool_number="60")
        ct = _simple_rectangle_contour(711.2, 609.6)
        ops = PartOperations(workpiece=wp, vertical_drills=[dr], contours=[ct])
        mpr = self.writer.write(ops)
        assert "<102 \\BohrVert\\" in mpr
        # Coordinates are 180°-rotated: XA = W-x = 711.2-100 = 611.2, YA = H-y = 609.6-50 = 559.6
        assert 'XA="611.2"' in mpr
        assert 'YA="559.6"' in mpr
        assert 'DU="5"' in mpr

    def test_blind_pocket_uses_freeform_block(self):
        from src.toolpath.operations import PocketOperation
        wp  = WorkpieceSpec(300.0, 200.0, 19.0, "T", "B", "f.dxf")
        ct  = _simple_rectangle_contour(300.0, 200.0, cid=1)
        pk_segs = [
            LineSegment((50.0, 50.0), (100.0, 50.0)),
            LineSegment((100.0, 50.0), (100.0, 100.0)),
            LineSegment((100.0, 100.0), (50.0, 100.0)),
            LineSegment((50.0, 100.0), (50.0, 50.0)),
        ]
        pk = PocketOperation(segments=pk_segs, depth_mm=9.5, contour_id=2,
                             tool_number="132", feed_rate=10.0, workstations="1,3,4")
        ops = PartOperations(workpiece=wp, contours=[ct], pockets=[pk])
        mpr = self.writer.write(ops)
        assert "<181 \\FreiFormTasche\\" in mpr
        assert 'TI="9.5"' in mpr
        assert 'T_="132"' in mpr
        assert 'RK="WRKL"' not in mpr

    def test_arc_segment_generates_ka(self):
        arc = ArcSegment(start=(0.0, 0.0), end=(10.0, 0.0),
                         center=(5.0, 0.0), radius=5.0, ccw=True)
        ct = ContourOperation(
            segments=[arc], depth_mm=5.0, rk="WRKL", contour_id=1,
            tool_number="101", feed_rate=10.0, workstations="1,2,3",
        )
        wp  = WorkpieceSpec(100.0, 100.0, 19.0, "T", "B", "f.dxf")
        ops = PartOperations(workpiece=wp, contours=[ct])
        mpr = self.writer.write(ops)
        assert "KA" in mpr
        assert "DS=3" in mpr   # CCW arc
        assert f"R={5.0}" in mpr or "R=5" in mpr

    def test_za_is_positive_depth(self):
        ops = _make_ops()
        mpr = self.writer.write(ops)
        # Depth is 20.1; ZA should be positive 20.1 (WoodWOP CadCamLT format)
        assert 'ZA="20.1"' in mpr
