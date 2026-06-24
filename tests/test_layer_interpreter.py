"""Tests for the layer interpreter."""

import pytest
from src.parser.layer_interpreter import interpret_layer, LayerType


class TestWorkpieceLayer:
    def test_basic(self):
        info = interpret_layer("ProcPart_19.1")
        assert info.layer_type == LayerType.WORKPIECE
        assert abs(info.thickness_mm - 19.1) < 0.001

    def test_underscore_decimal(self):
        info = interpret_layer("ProcPart_18_7")
        assert info.layer_type == LayerType.WORKPIECE
        assert abs(info.thickness_mm - 18.7) < 0.001

    def test_integer_thickness(self):
        info = interpret_layer("ProcPart_18")
        assert info.layer_type == LayerType.WORKPIECE
        assert info.thickness_mm == 18.0

    def test_case_insensitive(self):
        info = interpret_layer("PROCPART_19.1")
        assert info.layer_type == LayerType.WORKPIECE


class TestOuterContourLayer:
    def test_exact_match(self):
        info = interpret_layer("V_Fraes_2T134R")
        assert info.layer_type == LayerType.OUTER_CONTOUR
        assert info.depth_mm == 2.0
        assert info.tool_number == "134"

    def test_case_insensitive(self):
        info = interpret_layer("v_fraes_2t134r")
        assert info.layer_type == LayerType.OUTER_CONTOUR
        assert info.tool_number == "134"

    def test_plywood_tool(self):
        info = interpret_layer("V_Fraes_2T137R")
        assert info.layer_type == LayerType.OUTER_CONTOUR
        assert info.depth_mm == 2.0
        assert info.tool_number == "137"


class TestDrillLayer:
    def test_through(self):
        info = interpret_layer("V_DrillSF_19.1")
        assert info.layer_type == LayerType.VERTICAL_DRILL
        assert abs(info.depth_mm - 19.1) < 0.001

    def test_blind(self):
        info = interpret_layer("V_DrillSF_14")
        assert info.layer_type == LayerType.VERTICAL_DRILL
        assert info.depth_mm == 14.0

    def test_blind_decimal(self):
        info = interpret_layer("V_DrillSF_12")
        assert info.layer_type == LayerType.VERTICAL_DRILL
        assert info.depth_mm == 12.0


class TestPocketLayer:
    def test_left_comp(self):
        info = interpret_layer("V_Fraes_9.1T134L")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert abs(info.depth_mm - 9.1) < 0.001

    def test_through_pocket(self):
        info = interpret_layer("V_Fraes_0T134L")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert info.depth_mm == 0.0
        assert info.is_through

    def test_right_comp(self):
        info = interpret_layer("V_Fraes_5T134R")
        assert info.layer_type == LayerType.POCKET_RIGHT

    def test_underscore_decimal(self):
        info = interpret_layer("V_Fraes_9_5T134L")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert abs(info.depth_mm - 9.5) < 0.001

    def test_plywood_through_milling(self):
        info = interpret_layer("V_Fraes_0T137L")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert info.depth_mm == 0.0
        assert info.is_through
        assert info.tool_number == "137"

    def test_nonplywood_through_milling(self):
        info = interpret_layer("V_Fraes_0T132L")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert info.depth_mm == 0.0
        assert info.is_through
        assert info.tool_number == "132"


class TestFPocketLayer:
    def test_integer_depth(self):
        info = interpret_layer("F_Pocket_5")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert info.depth_mm == 5.0
        assert not info.is_through

    def test_decimal_depth(self):
        info = interpret_layer("F_Pocket_12.5")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert abs(info.depth_mm - 12.5) < 0.001

    def test_underscore_decimal(self):
        info = interpret_layer("F_Pocket_12_5")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert abs(info.depth_mm - 12.5) < 0.001

    def test_case_insensitive(self):
        info = interpret_layer("f_pocket_8")
        assert info.layer_type == LayerType.POCKET_LEFT
        assert info.depth_mm == 8.0


class TestIgnoredLayers:
    def test_ocl_text(self):
        info = interpret_layer("OCL_TEXT")
        assert info.layer_type == LayerType.TEXT

    def test_ocl_part(self):
        info = interpret_layer("OCL_PART")
        assert info.layer_type == LayerType.IGNORED

    def test_unknown(self):
        info = interpret_layer("RANDOM_LAYER")
        assert info.layer_type == LayerType.IGNORED
