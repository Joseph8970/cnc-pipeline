"""
Integration tests – parse real DXF files and verify end-to-end sheet-level output.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from src.parser.dxf_parser import DXFParser, RawSheet
from src.toolpath.toolpath_engine import ToolpathEngine
from src.validation.validator import PartValidator
from src.writer.mpr_writer import MPRWriter


@pytest.fixture
def pipeline(cfg):
    return {
        "parser":    DXFParser(),
        "engine":    ToolpathEngine(cfg),
        "validator": PartValidator(),
        "writer":    MPRWriter(
            woodwop_version=cfg.mpr.woodwop_version,
            mpr_version=cfg.mpr.version,
            material=cfg.mpr.material,
            profile=cfg.mpr.profile,
        ),
    }


class TestParseSheet1:
    def test_sheet_dimensions(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        assert sheet.sheet_width  > 0
        assert sheet.sheet_height > 0
        assert sheet.thickness_mm > 0

    def test_standard_sheet_size(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        # Standard 4×8 sheet = 2438.4 × 1219.2 mm
        assert abs(sheet.sheet_width  - 2438.4) < 1.0
        assert abs(sheet.sheet_height - 1219.2) < 1.0

    def test_outer_contours_present(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        outer = [p for p in sheet.polylines if p.layer == "V_Fraes_2T134R"]
        assert outer, "No outer-contour polylines found"

    def test_world_coords_within_sheet(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        for c in sheet.circles:
            assert -10 <= c.cx <= sheet.sheet_width  + 10, f"Circle X={c.cx} outside sheet"
            assert -10 <= c.cy <= sheet.sheet_height + 10, f"Circle Y={c.cy} outside sheet"


class TestParseSheet2:
    def test_drills_in_world_coords(self, pipeline, sample_dxf_path2):
        sheet = pipeline["parser"].parse_file(sample_dxf_path2)
        drills = [c for c in sheet.circles if "DrillSF" in c.layer]
        assert drills, "No drill circles in sheet 2"
        for d in drills:
            assert d.cx >= 0 and d.cy >= 0, "Drill at negative world coords"

    def test_pocket_polylines_present(self, pipeline, sample_dxf_path2):
        sheet = pipeline["parser"].parse_file(sample_dxf_path2)
        pockets = [p for p in sheet.polylines if "T134L" in p.layer]
        assert pockets, "No pocket polylines found in sheet 2"


class TestToolpathSheet1:
    def test_workpiece_equals_sheet(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        ops   = pipeline["engine"].process(sheet)
        assert abs(ops.workpiece.width_x - sheet.sheet_width)  < 0.01
        assert abs(ops.workpiece.width_y - sheet.sheet_height) < 0.01

    def test_outer_contour_ops_created(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        ops   = pipeline["engine"].process(sheet)
        outer = [c for c in ops.contours if c.is_outer]
        assert outer, "No outer ContourOperation generated"

    def test_validation_passes(self, pipeline, sample_dxf_path):
        sheet  = pipeline["parser"].parse_file(sample_dxf_path)
        ops    = pipeline["engine"].process(sheet)
        result = pipeline["validator"].validate(ops)
        assert result.is_valid, f"Validation failed:\n{result}"


class TestToolpathSheet2:
    def test_drills_non_negative(self, pipeline, sample_dxf_path2):
        sheet = pipeline["parser"].parse_file(sample_dxf_path2)
        ops   = pipeline["engine"].process(sheet)
        for dr in ops.vertical_drills:
            assert dr.x >= 0 and dr.y >= 0
            assert dr.depth > 0

    def test_pockets_have_left_comp(self, pipeline, sample_dxf_path2):
        sheet = pipeline["parser"].parse_file(sample_dxf_path2)
        ops   = pipeline["engine"].process(sheet)
        all_left = [c for c in ops.contours if c.rk == "WRKL"] + list(ops.pockets)
        assert all_left, "No left-compensation operations in sheet 2"


class TestMPROutput:
    def test_mpr_starts_with_header(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        ops   = pipeline["engine"].process(sheet)
        mpr   = pipeline["writer"].write(ops)
        assert mpr.startswith("[H")

    def test_mpr_ends_with_exclamation(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        ops   = pipeline["engine"].process(sheet)
        mpr   = pipeline["writer"].write(ops)
        assert mpr.rstrip().endswith("!")

    def test_bsx_equals_sheet_width(self, pipeline, sample_dxf_path):
        sheet = pipeline["parser"].parse_file(sample_dxf_path)
        ops   = pipeline["engine"].process(sheet)
        mpr   = pipeline["writer"].write(ops)
        m = re.search(r"_BSX=([\d.]+)", mpr)
        assert m
        assert abs(float(m.group(1)) - sheet.sheet_width) < 0.01

    def test_drill_count_matches(self, pipeline, sample_dxf_path2):
        sheet = pipeline["parser"].parse_file(sample_dxf_path2)
        ops   = pipeline["engine"].process(sheet)
        mpr   = pipeline["writer"].write(ops)
        assert mpr.count("<102 \\BohrVert\\") == len(ops.vertical_drills)

    def test_one_mpr_per_sheet_all_files(self, pipeline, cfg):
        dxf_files = sorted(cfg.paths.to_woodwop.glob("*.dxf"))
        if not dxf_files:
            pytest.skip("No DXF files in to_woodwop/")

        for dxf in dxf_files:
            sheet  = pipeline["parser"].parse_file(dxf)
            ops    = pipeline["engine"].process(sheet)
            result = pipeline["validator"].validate(ops)
            assert result.is_valid, f"{dxf.name}:\n{result}"
            mpr = pipeline["writer"].write(ops)
            assert "[H" in mpr, f"No header in MPR for {dxf.name}"

        print(f"\nProcessed {len(dxf_files)} sheet(s) → {len(dxf_files)} MPR(s)")
