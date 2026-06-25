"""Tests for arc_repair.py — OpenCutList fragmented-arc merging."""

import math
import pytest
import ezdxf

from arc_repair import analyze_arc_chains, repair_opencutlist_arcs


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_doc():
    return ezdxf.new("R2010")


def _add_arc(layout, cx, cy, r, sa, ea, layer="0"):
    return layout.add_arc(center=(cx, cy, 0), radius=r,
                          start_angle=sa, end_angle=ea,
                          dxfattribs={"layer": layer})


def _arc_entities(layout):
    return list(layout.query("ARC"))


def _line_entities(layout):
    return list(layout.query("LINE"))


# ---------------------------------------------------------------------------
# Two consecutive co-circular arcs → merge to one
# ---------------------------------------------------------------------------

class TestTwoArcMerge:
    def test_two_arcs_on_same_circle_merge(self):
        doc = _make_doc()
        msp = doc.modelspace()
        # ARC1: 0°→45°   ARC2: 45°→90°   both on unit circle centred at origin
        _add_arc(msp, 0, 0, 1.0,  0.0, 45.0)
        _add_arc(msp, 0, 0, 1.0, 45.0, 90.0)

        n = repair_opencutlist_arcs(doc)

        assert n == 1
        arcs = _arc_entities(msp)
        assert len(arcs) == 1
        assert abs(arcs[0].dxf.start_angle -  0.0) < 1e-6
        assert abs(arcs[0].dxf.end_angle   - 90.0) < 1e-6
        assert abs(arcs[0].dxf.radius - 1.0) < 1e-9

    def test_layer_preserved_from_first_arc(self):
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp, 0, 0, 5.0,  0.0, 60.0, layer="V_Fraes_2T137R")
        _add_arc(msp, 0, 0, 5.0, 60.0, 120.0, layer="V_Fraes_2T137R")

        repair_opencutlist_arcs(doc)

        arcs = _arc_entities(msp)
        assert len(arcs) == 1
        assert arcs[0].dxf.layer == "V_Fraes_2T137R"

    def test_three_consecutive_arcs_merge_to_one(self):
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp, 0, 0, 10.0,   0.0,  30.0)
        _add_arc(msp, 0, 0, 10.0,  30.0,  60.0)
        _add_arc(msp, 0, 0, 10.0,  60.0,  90.0)

        n = repair_opencutlist_arcs(doc)

        assert n == 1
        arcs = _arc_entities(msp)
        assert len(arcs) == 1
        assert abs(arcs[0].dxf.start_angle -  0.0) < 1e-6
        assert abs(arcs[0].dxf.end_angle   - 90.0) < 1e-6


# ---------------------------------------------------------------------------
# Arcs on DIFFERENT circles must NOT be merged
# ---------------------------------------------------------------------------

class TestNonMerge:
    def test_different_radius_not_merged(self):
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp, 0, 0, 1.0,  0.0, 45.0)
        _add_arc(msp, 0, 0, 2.0, 45.0, 90.0)   # different radius

        n = repair_opencutlist_arcs(doc)

        assert n == 0
        assert len(_arc_entities(msp)) == 2

    def test_different_center_not_merged(self):
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp,  0,  0, 1.0,  0.0, 45.0)
        _add_arc(msp, 10, 10, 1.0, 45.0, 90.0)  # different center

        n = repair_opencutlist_arcs(doc)

        assert n == 0
        assert len(_arc_entities(msp)) == 2

    def test_non_connected_endpoints_not_merged(self):
        # Same circle but gap between endpoints exceeds _GAP_TOL (0.05 mm).
        # On a unit circle, a 5° angular gap ≈ 0.087 mm > 0.05 mm.
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp, 0, 0, 1.0,  0.0, 40.0)   # ends at 40°
        _add_arc(msp, 0, 0, 1.0, 45.0, 90.0)   # starts at 45° — gap ≈ 0.087 mm

        n = repair_opencutlist_arcs(doc)
        assert n == 0


# ---------------------------------------------------------------------------
# Tiny LINE connector between two co-circular arcs → absorbed
# ---------------------------------------------------------------------------

class TestLineConnector:
    def _arc_pt(self, cx, cy, r, angle_deg):
        a = math.radians(angle_deg)
        return (cx + r * math.cos(a), cy + r * math.sin(a))

    def test_tiny_line_bridge_absorbed(self):
        doc = _make_doc()
        msp = doc.modelspace()
        cx, cy, r = 0.0, 0.0, 50.0

        _add_arc(msp, cx, cy, r,  0.0, 45.0)
        # Tiny connector line at the junction (45°)
        pt = self._arc_pt(cx, cy, r, 45.0)
        msp.add_line(pt, pt, dxfattribs={"layer": "0"})   # zero-length (degenerate) is fine
        _add_arc(msp, cx, cy, r, 45.0, 90.0)

        n = repair_opencutlist_arcs(doc)

        assert n == 1
        assert len(_arc_entities(msp)) == 1
        assert len(_line_entities(msp)) == 0

    def test_large_line_not_absorbed(self):
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp, 0, 0, 50.0,  0.0, 45.0)
        # A 10 mm line — well above tiny_line_mm default of 1.0
        pt45 = (50 * math.cos(math.radians(45)), 50 * math.sin(math.radians(45)))
        far   = (pt45[0] + 10.0, pt45[1])
        msp.add_line(pt45, far, dxfattribs={"layer": "0"})
        _add_arc(msp, 0, 0, 50.0, 45.0, 90.0)

        n = repair_opencutlist_arcs(doc)
        assert n == 0
        assert len(_arc_entities(msp)) == 2
        assert len(_line_entities(msp)) == 1


# ---------------------------------------------------------------------------
# Block definitions (not just modelspace)
# ---------------------------------------------------------------------------

class TestBlockRepair:
    def test_arcs_in_block_definition_merged(self):
        doc = _make_doc()
        blk = doc.blocks.new("OCL_PART___A")
        _add_arc(blk, 0, 0, 30.0,  0.0, 90.0, layer="V_Fraes_2T137R")
        _add_arc(blk, 0, 0, 30.0, 90.0, 180.0, layer="V_Fraes_2T137R")

        n = repair_opencutlist_arcs(doc)

        assert n == 1
        arcs = list(blk.query("ARC"))
        assert len(arcs) == 1
        assert abs(arcs[0].dxf.start_angle -   0.0) < 1e-6
        assert abs(arcs[0].dxf.end_angle   - 180.0) < 1e-6


# ---------------------------------------------------------------------------
# analyze_arc_chains must not raise (smoke test)
# ---------------------------------------------------------------------------

class TestAnalyze:
    def test_analyze_does_not_raise(self, capsys):
        doc = _make_doc()
        msp = doc.modelspace()
        _add_arc(msp, 0, 0, 1.0,  0.0, 45.0)
        _add_arc(msp, 0, 0, 1.0, 45.0, 90.0)

        analyze_arc_chains(doc)   # must not raise

        out = capsys.readouterr().out
        assert "Chain #1" in out
        assert "2 arcs" in out
