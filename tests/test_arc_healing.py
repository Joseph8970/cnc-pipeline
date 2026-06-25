"""Tests for arc_healing.py (no ezdxf dependency for unit tests)."""

import math
import pytest

from arc_healing import (
    ArcHealConfig,
    _arc_sample_points,
    _arc_sweep_deg,
    _build_chains,
    _dist,
    _entity_chord,
    _entity_endpoints,
    _fit_circle,
    _max_residual,
)


# ---------------------------------------------------------------------------
# Helpers: lightweight stubs that mimic ezdxf entity behaviour
# ---------------------------------------------------------------------------

class _Vec:
    def __init__(self, x, y, z=0.0):
        self.x = x; self.y = y; self.z = z


class _Dxf:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def hasattr(self, name):
        return name in self.__dict__


class _Arc:
    def __init__(self, cx, cy, r, sa, ea, layer="0"):
        self.dxf = _Dxf(
            center=_Vec(cx, cy),
            radius=r,
            start_angle=sa,
            end_angle=ea,
            layer=layer,
        )
        self._destroyed = False

    def dxftype(self):
        return "ARC"

    def destroy(self):
        self._destroyed = True


class _Line:
    def __init__(self, x1, y1, x2, y2, layer="0"):
        self.dxf = _Dxf(
            start=_Vec(x1, y1),
            end=_Vec(x2, y2),
            layer=layer,
        )
        self._destroyed = False

    def dxftype(self):
        return "LINE"

    def destroy(self):
        self._destroyed = True


class _Other:
    def dxftype(self):
        return "CIRCLE"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

class TestEntityHelpers:
    def test_arc_endpoints_unit_circle(self):
        arc = _Arc(0, 0, 1, 0, 90)
        sp, ep = _entity_endpoints(arc)
        assert abs(sp[0] - 1.0) < 1e-9 and abs(sp[1] - 0.0) < 1e-9
        assert abs(ep[0] - 0.0) < 1e-6 and abs(ep[1] - 1.0) < 1e-6

    def test_line_endpoints(self):
        line = _Line(1, 2, 3, 4)
        sp, ep = _entity_endpoints(line)
        assert sp == (1.0, 2.0)
        assert ep == (3.0, 4.0)

    def test_other_entity_returns_none(self):
        assert _entity_endpoints(_Other()) is None

    def test_arc_chord_quarter_unit_circle(self):
        arc = _Arc(0, 0, 1, 0, 90)
        # chord from (1,0) to (0,1) = sqrt(2)
        assert abs(_entity_chord(arc) - math.sqrt(2)) < 1e-6

    def test_line_chord(self):
        line = _Line(0, 0, 3, 4)
        assert abs(_entity_chord(line) - 5.0) < 1e-9

    def test_arc_sweep_full_circle_wraps(self):
        arc = _Arc(0, 0, 1, 270, 90)   # 0→90 CCW with wrap
        assert abs(_arc_sweep_deg(arc) - 180.0) < 1e-9

    def test_arc_sample_points_count(self):
        # Short arc (<60°): 3 points (start, end, mid)
        arc = _Arc(0, 0, 100, 0, 30)
        pts = _arc_sample_points(arc)
        assert len(pts) == 3

        # Long arc (>60°): 5 points
        arc2 = _Arc(0, 0, 100, 0, 90)
        pts2 = _arc_sample_points(arc2)
        assert len(pts2) == 5


# ---------------------------------------------------------------------------
# Circle fitting
# ---------------------------------------------------------------------------

class TestFitCircle:
    def test_unit_circle_three_points(self):
        pts = [(1, 0), (0, 1), (-1, 0)]
        result = _fit_circle(pts)
        assert result is not None
        cx, cy, r = result
        assert abs(cx) < 1e-6
        assert abs(cy) < 1e-6
        assert abs(r - 1.0) < 1e-6

    def test_offset_circle(self):
        R = 50.0; cx0 = 100.0; cy0 = 200.0
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        pts = [(cx0 + R * math.cos(math.radians(a)),
                cy0 + R * math.sin(math.radians(a))) for a in angles]
        result = _fit_circle(pts)
        assert result is not None
        cx, cy, r = result
        assert abs(cx - cx0) < 1e-4
        assert abs(cy - cy0) < 1e-4
        assert abs(r - R) < 1e-4

    def test_collinear_points_returns_none(self):
        pts = [(0, 0), (1, 0), (2, 0)]
        # Collinear → degenerate
        result = _fit_circle(pts)
        assert result is None or (result is not None and result[2] > 1e6)

    def test_fewer_than_3_points(self):
        assert _fit_circle([]) is None
        assert _fit_circle([(1, 0)]) is None
        assert _fit_circle([(1, 0), (-1, 0)]) is None

    def test_max_residual_exact_circle(self):
        R = 10.0
        pts = [(R * math.cos(math.radians(a)), R * math.sin(math.radians(a)))
               for a in range(0, 360, 45)]
        assert _max_residual(pts, 0.0, 0.0, R) < 1e-9

    def test_max_residual_off_point(self):
        R = 10.0
        pts = [(R, 0), (0, R), (-R, 0), (0, R * 1.1)]  # last point deviates
        err = _max_residual(pts, 0.0, 0.0, R)
        assert err == pytest.approx(R * 0.1, abs=1e-6)


# ---------------------------------------------------------------------------
# Chain building
# ---------------------------------------------------------------------------

class TestBuildChains:
    def _cfg(self):
        c = ArcHealConfig()
        c.endpoint_tol_mm = 0.05
        c.max_connector_mm = 0.25
        return c

    def _arc_on_unit(self, sa, ea):
        """Build an ARC entity on the unit circle."""
        return _Arc(0, 0, 1, sa, ea)

    def test_two_direct_arcs_chain(self):
        # Arc 0°→90° then 90°→180°: end of first = start of second
        a1 = self._arc_on_unit(0, 90)
        a2 = self._arc_on_unit(90, 180)
        cfg = self._cfg()
        chains = _build_chains([a1, a2], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        assert len(chains) == 1
        assert len(chains[0]) == 2

    def test_two_arcs_with_tiny_line_bridge(self):
        # a1 ends at (0, 1) — point at 90° on unit circle centred at origin
        a1 = self._arc_on_unit(0, 90)
        _, ep = _entity_endpoints(a1)  # ep ≈ (0, 1)

        # tiny bridge from (0, 1) to (0.05, 1) — 0.05 mm < max_connector_mm
        bx, by = ep[0] + 0.05, ep[1]
        bridge = _Line(ep[0], ep[1], bx, by)

        # a2 must start exactly at (bx, by).
        # Use centre (0.05, 0) radius 1 → start_angle 90° gives point
        # (0.05 + 1*cos90, 0 + 1*sin90) = (0.05, 1.0) ✓
        a2 = _Arc(0.05, 0.0, 1.0, 90.0, 180.0)

        cfg = self._cfg()
        chains = _build_chains([a1, bridge, a2], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        # Bridge chord 0.05 mm < 0.25 → should be absorbed; chain has 2 ARCs
        total_arcs = sum(sum(1 for e in ch if e.dxftype() == "ARC") for ch in chains)
        assert total_arcs >= 2

    def test_large_line_breaks_chain(self):
        a1 = self._arc_on_unit(0, 90)
        ep = _entity_endpoints(a1)[1]
        big_line = _Line(ep[0], ep[1], ep[0] + 10.0, ep[1])  # 10 mm >> 0.25
        a2 = self._arc_on_unit(90, 180)
        cfg = self._cfg()
        chains = _build_chains([a1, big_line, a2], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        # No single chain should span both ARCs across the big line
        for ch in chains:
            arc_count = sum(1 for e in ch if e.dxftype() == "ARC")
            assert arc_count < 2 or big_line not in ch

    def test_single_arc_not_returned(self):
        a1 = self._arc_on_unit(0, 90)
        cfg = self._cfg()
        chains = _build_chains([a1], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        assert chains == []

    def test_different_layers_not_chained(self):
        a1 = _Arc(0, 0, 1, 0, 90, layer="LayerA")
        a2 = _Arc(0, 0, 1, 90, 180, layer="LayerB")
        cfg = self._cfg()
        chains = _build_chains([a1, a2], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        assert chains == []

    def test_branch_point_stops_chain(self):
        # Two arcs both starting at (0,1): a2 AND a3
        a1 = self._arc_on_unit(0, 90)
        a2 = self._arc_on_unit(90, 180)
        # a3 also starts at (0,1) — creates a branch
        a3 = _Arc(0, 0, 1, 90, 135)
        cfg = self._cfg()
        chains = _build_chains([a1, a2, a3], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        # a1 → a2 and a1 → a3 both valid successors → ambiguous → no chain for a1
        for ch in chains:
            assert a1 not in ch  # a1 should not start any chain

    def test_non_arc_entities_ignored(self):
        a1 = self._arc_on_unit(0, 90)
        a2 = self._arc_on_unit(90, 180)
        other = _Other()
        cfg = self._cfg()
        chains = _build_chains([other, a1, a2, other], cfg.endpoint_tol_mm, cfg.max_connector_mm)
        assert len(chains) == 1
