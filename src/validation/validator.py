"""
Pre-write validation.

Validates a ``PartOperations`` object before MPR generation and returns a
``ValidationResult`` with a list of errors and warnings.  The caller decides
whether to abort on errors.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from src.toolpath.operations import PartOperations, ContourOperation


@dataclass
class ValidationResult:
    errors:   List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

    def add_error(self, msg: str) -> None:
        self.errors.append(msg)

    def add_warning(self, msg: str) -> None:
        self.warnings.append(msg)

    def __str__(self) -> str:
        lines = []
        for e in self.errors:
            lines.append(f"  ERROR:   {e}")
        for w in self.warnings:
            lines.append(f"  WARNING: {w}")
        return "\n".join(lines) if lines else "  OK"


class PartValidator:
    """Validates a PartOperations object before MPR generation."""

    _MIN_DIMENSION_MM = 1.0
    _MAX_DIMENSION_MM = 5000.0
    _MIN_DEPTH_MM     = 0.5
    _MIN_DRILL_DIAM   = 1.0

    def validate(self, ops: PartOperations) -> ValidationResult:
        result = ValidationResult()
        wp = ops.workpiece

        # ------------------------------------------------------------------
        # Workpiece
        # ------------------------------------------------------------------
        if wp.width_x < self._MIN_DIMENSION_MM or wp.width_x > self._MAX_DIMENSION_MM:
            result.add_error(
                f"Workpiece width_x {wp.width_x:.2f} mm is out of range "
                f"[{self._MIN_DIMENSION_MM}, {self._MAX_DIMENSION_MM}]"
            )
        if wp.width_y < self._MIN_DIMENSION_MM or wp.width_y > self._MAX_DIMENSION_MM:
            result.add_error(
                f"Workpiece width_y {wp.width_y:.2f} mm is out of range "
                f"[{self._MIN_DIMENSION_MM}, {self._MAX_DIMENSION_MM}]"
            )
        if wp.thickness_z < self._MIN_DEPTH_MM or wp.thickness_z > 100.0:
            result.add_error(
                f"Workpiece thickness {wp.thickness_z:.2f} mm is out of range"
            )

        # ------------------------------------------------------------------
        # Contours
        # ------------------------------------------------------------------
        outer_contours = [c for c in ops.contours if c.is_outer]
        if not outer_contours:
            result.add_warning("No outer contour found – sheet may have no routing")

        for i, ct in enumerate(ops.contours):
            self._validate_contour(ct, f"contour[{i}]", result)

        for i, pk in enumerate(ops.pockets):
            self._validate_contour(pk.to_contour_operation(), f"pocket[{i}]", result)

        # ------------------------------------------------------------------
        # Drills
        # ------------------------------------------------------------------
        for i, dr in enumerate(ops.vertical_drills):
            if dr.diameter < self._MIN_DRILL_DIAM:
                result.add_error(
                    f"Vertical drill [{i}] diameter {dr.diameter:.2f} mm is too small"
                )
            if dr.depth < self._MIN_DEPTH_MM:
                result.add_error(
                    f"Vertical drill [{i}] depth {dr.depth:.2f} mm is too small"
                )
            if not (0 <= dr.x <= wp.width_x + 10) or not (0 <= dr.y <= wp.width_y + 10):
                result.add_warning(
                    f"Vertical drill [{i}] at ({dr.x:.2f}, {dr.y:.2f}) may be "
                    f"outside workpiece boundary"
                )

        # ------------------------------------------------------------------
        # Duplicate detection
        # ------------------------------------------------------------------
        drill_positions = [(round(d.x, 1), round(d.y, 1)) for d in ops.vertical_drills]
        if len(drill_positions) != len(set(drill_positions)):
            result.add_warning("Duplicate drill positions detected")

        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _validate_contour(
        self,
        ct: ContourOperation,
        label: str,
        result: ValidationResult,
    ) -> None:
        if not ct.segments:
            result.add_error(f"{label}: contour has no segments")
            return
        if len(ct.segments) < 2:
            result.add_warning(f"{label}: contour has only {len(ct.segments)} segment(s)")

        if ct.depth_mm < 0:
            result.add_error(f"{label}: depth {ct.depth_mm:.2f} mm is negative")

        # Check closure: last segment end ≈ first segment start
        import math
        start = ct.segments[0].start
        end   = ct.segments[-1].end
        gap   = math.hypot(end[0] - start[0], end[1] - start[1])
        if gap > 0.5:
            result.add_warning(
                f"{label}: contour gap of {gap:.3f} mm between last and first point"
            )
