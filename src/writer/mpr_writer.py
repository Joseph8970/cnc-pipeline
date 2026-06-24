"""
MPR writer -- WoodWOP 9 / CadCamLT format.
"""

from __future__ import annotations

from datetime import datetime
from io import StringIO
from pathlib import Path
from typing import List

from src.geometry.geometry_engine import ArcSegment, LineSegment
from src.toolpath.operations import (
    ContourOperation,
    PartOperations,
    PocketOperation,
    VerticalDrillOperation,
    WorkpieceSpec,
)


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _f(v: float, decimals: int = 6) -> str:
    return f"{v:.{decimals}f}"


def _fmm(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return s if s else "0"


def _fdelta(v: float) -> str:
    return "@" + _fmm(v)


# ---------------------------------------------------------------------------
# MPR Writer
# ---------------------------------------------------------------------------

class MPRWriter:

    def __init__(
        self,
        woodwop_version: str   = "9.0.152",
        mpr_version:     str   = "4.0 Alpha",
        material:        str   = "HOMAG",
        profile:         str   = "CadCamLT",
        fnx:             float = 0.0,
        fny:             float = 0.0,
    ) -> None:
        self._ww_ver   = woodwop_version
        self._mpr_ver  = mpr_version
        self._material = material
        self._profile  = profile
        self._fnx      = fnx
        self._fny      = fny

    # ------------------------------------------------------------------
    # Public entry points
    # ------------------------------------------------------------------

    def write(self, ops: PartOperations) -> str:
        buf = StringIO()
        wp  = ops.workpiece

        # Classify contours
        through_contours: List[ContourOperation] = []  # WRKL through-cuts → <105>
        outer_contours:   List[ContourOperation] = []  # WRKR outer routing → <105>
        for ct in ops.contours:
            (outer_contours if ct.rk == "WRKR" else through_contours).append(ct)

        # Blind pockets → <181 \FreiFormTasche\>; build proxy ContourOperations for definitions
        freeform_pockets: List[PocketOperation]  = list(ops.pockets)
        pocket_defs:      List[ContourOperation] = [pk.to_contour_operation() for pk in freeform_pockets]

        # Assign sequential contour IDs: freeform pockets, then through-cuts, then outer
        all_defs = pocket_defs + through_contours + outer_contours
        for idx, ct in enumerate(all_defs, start=1):
            ct.contour_id = idx
        for pk, ct in zip(freeform_pockets, pocket_defs):
            pk.contour_id = ct.contour_id

        now = datetime.now()
        buf.write(self._header(wp, now))

        for ct in all_defs:
            buf.write(self._contour_definition(ct, wp))

        buf.write(self._workpiece_block(wp))
        buf.write(self._comment_block(wp, now))

        # Global ORI counter; comment block = ORI 1
        ori = 2

        # 1. Blind pockets → <181 \FreiFormTasche\>
        for pk in freeform_pockets:
            buf.write(self._freeform_pocket_block(pk, ori))
            ori += 1

        # 2. Through-cuts → <105> WRKL
        for ct in through_contours:
            buf.write(self._contour_routing_block(ct, ori))
            ori += 1

        # 3. Outer contour scoring passes → <105> WRKR
        for ct in outer_contours:
            buf.write(self._contour_routing_block(ct, ori))
            ori += 1

        # 4. T130 finish passes → <105> WRKR (same contour_id, different TNO/ZA)
        for ct in outer_contours:
            if ct.through_pass_tool_number:
                through = ContourOperation(
                    segments=ct.segments,
                    depth_mm=ct.through_pass_depth,
                    rk=ct.rk,
                    contour_id=ct.contour_id,
                    tool_number=ct.through_pass_tool_number,
                    feed_rate=ct.feed_rate,
                    workstations=ct.workstations,
                )
                buf.write(self._contour_routing_block(through, ori))
                ori += 1

        # 5. Vertical drills (grouped by equal X-spacing where possible)
        for dr, xa, ya, count, step in self._group_drills(ops.vertical_drills, wp):
            buf.write(self._vertical_drill_block(dr, ori, wp, xa=xa, ya=ya, count=count, step=step))
            ori += 1

        buf.write("!\n")
        return buf.getvalue()

    def write_file(self, ops: PartOperations, out_path: Path) -> None:
        """Write MPR using Windows-1252 encoding with CRLF (WoodWOP requirement)."""
        content = self.write(ops)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_bytes(content.encode("cp1252").replace(b"\n", b"\r\n"))

    # ------------------------------------------------------------------
    # Section builders
    # ------------------------------------------------------------------

    def _header(self, wp: WorkpieceSpec, now: datetime) -> str:
        rx = _f(wp.width_x + self._fnx * 2)
        ry = _f(wp.width_y + self._fny * 2)
        return (
            f"[H\n"
            f'VERSION="{self._mpr_ver}"\n'
            f'WW="{self._ww_ver}"\n'
            f'OP="1"\n'
            f'WRK2="0"\n'
            f'SCHN="0"\n'
            f'CVR="0"\n'
            f'POI="0"\n'
            f'HSP="0"\n'
            f'O2="0"\n'
            f'O4="0"\n'
            f'O3="0"\n'
            f'O5="0"\n'
            f'SR="0"\n'
            f'FM="1"\n'
            f'ML="2000"\n'
            f'UF="20"\n'
            f'ZS="20"\n'
            f'DN="STANDARD"\n'
            f'DST="0"\n'
            f'GP="0"\n'
            f'GY="0"\n'
            f'GXY="0"\n'
            f'NP="1"\n'
            f'NE="0"\n'
            f'NA="0"\n'
            f'BFS="1"\n'
            f'US="0"\n'
            f'CB="0"\n'
            f'UP="0"\n'
            f'DW="0"\n'
            f'MAT="{self._material}"\n'
            f'HP_A_O="STANDARD"\n'
            f'OVD_U="1"\n'
            f'OVD="0"\n'
            f'OHD_U="0"\n'
            f'OHD="2"\n'
            f'OOMD_U="0"\n'
            f'EWL="1"\n'
            f'INCH="0"\n'
            f'VIEW="NOMIRROR"\n'
            f'ANZ="1"\n'
            f'BES="0"\n'
            f'ENT="0"\n'
            f'MATERIAL=""\n'
            f'CUSTOMER=""\n'
            f'ORDER=""\n'
            f'ARTICLE=""\n'
            f'PARTID=""\n'
            f'PARTTYPE=""\n'
            f'MPRCOUNT="1"\n'
            f'MPRNUMBER="1"\n'
            f'INFO1=""\n'
            f'INFO2=""\n'
            f'INFO3=""\n'
            f'INFO4=""\n'
            f'INFO5=""\n'
            f"_BSX={_f(wp.width_x)}\n"
            f"_BSY={_f(wp.width_y)}\n"
            f"_BSZ={_f(wp.thickness_z)}\n"
            f"_FNX={_f(self._fnx)}\n"
            f"_FNY={_f(self._fny)}\n"
            f"_RNX={_f(0.0)}\n"
            f"_RNY={_f(0.0)}\n"
            f"_RNZ={_f(0.0)}\n"
            f"_RX={rx}\n"
            f"_RY={ry}\n"
            f"\n"
        )

    def _contour_definition(self, ct: ContourOperation, wp: WorkpieceSpec) -> str:
        """Write a contour definition block.

        WoodWOP expects (0,0) at the corner opposite to where ezdxf/DXF places it,
        so every coordinate is flipped: x_mpr = BSX - x, y_mpr = BSY - y.
        This is a 180° rotation about the sheet centre, which preserves contour
        orientation (CW stays CW) so WRKR/WRKL and arc MI flags need no change.
        """
        W, H = wp.width_x, wp.width_y

        def fx(x: float) -> float: return W - x
        def fy(y: float) -> float: return H - y

        n   = ct.contour_id
        buf = StringIO()
        buf.write(f"]{n}\n")

        if not ct.segments:
            return buf.getvalue()

        raw_sx, raw_sy = ct.segments[0].start
        start_x, start_y = fx(raw_sx), fy(raw_sy)

        buf.write("$E0\n")
        buf.write("KP \n")
        buf.write(f"X={_fmm(start_x)}\n")
        buf.write(f"Y={_fmm(start_y)}\n")
        buf.write("Z=0\n")
        buf.write("KO=00\n")
        buf.write(".X=0.000000\n")
        buf.write(".Y=0.000000\n")
        buf.write(".Z=0.000000\n")
        buf.write(".KO=00\n")
        buf.write("\n")

        # Track flipped coordinates for delta computation
        prev_x, prev_y = start_x, start_y

        for i, seg in enumerate(ct.segments, start=1):
            raw_ex, raw_ey = seg.end
            ex, ey = fx(raw_ex), fy(raw_ey)
            buf.write(f"$E{i}\n")

            if isinstance(seg, LineSegment):
                buf.write("KL \n")
                buf.write(f"X={_fdelta(ex - prev_x)}\n")
                buf.write(f"Y={_fdelta(ey - prev_y)}\n")
                buf.write("Z=@0\n")
                buf.write(".X=0.000000\n")
                buf.write(".Y=0.000000\n")
                buf.write(".Z=0.000000\n")
                buf.write(".WI=0.000000\n")
                buf.write(".WZ=0.000000\n")

            elif isinstance(seg, ArcSegment):
                ds = 3 if seg.ccw else 4
                buf.write("KA \n")
                buf.write(f"X={_fdelta(ex - prev_x)}\n")
                buf.write(f"Y={_fdelta(ey - prev_y)}\n")
                buf.write("Z=@0\n")
                buf.write(f"DS={ds}\n")
                buf.write(f"R={_fmm(seg.radius)}\n")
                buf.write(".X=0.000000\n")
                buf.write(".Y=0.000000\n")
                buf.write(".Z=0.000000\n")
                buf.write(".I=0.000000\n")
                buf.write(".J=0.000000\n")
                buf.write(".DS=0\n")
                buf.write(".R=0.000000\n")
                buf.write(".WI=0.000000\n")
                buf.write(".WO=0.000000\n")
                buf.write(".WAZ=0.000000\n")

            buf.write("\n")
            prev_x, prev_y = ex, ey

        return buf.getvalue()

    def _workpiece_block(self, wp: WorkpieceSpec) -> str:
        return (
            f"<100 \\WerkStck\\\n"
            f'LA="{_fmm(wp.width_x)}"\n'
            f'BR="{_fmm(wp.width_y)}"\n'
            f'DI="{_fmm(wp.thickness_z)}"\n'
            f'FNX="{_fmm(self._fnx)}"\n'
            f'FNY="{_fmm(self._fny)}"\n'
            f'AX="0"\n'
            f'AY="0"\n'
            f"\n"
        )

    def _comment_block(self, wp: WorkpieceSpec, now: datetime) -> str:
        date_str = now.strftime("%m/%d/%y")
        time_str = now.strftime("Uhrzeit:%H:%M:%S ")
        return (
            f"<101 \\Kommentar\\\n"
            f'KM="Masseinheit / Unit = Millimeter"\n'
            f'KM="{date_str}"\n'
            f'KM="{time_str}"\n'
            f'KM="Profil:{self._profile}"\n'
            f'KM="Quelle:{wp.source_file}"\n'
            f'KAT="Kommentar"\n'
            f'MNM="Kommentar"\n'
            f'ORI="1"\n'
            f"\n"
        )

    def _group_drills(
        self,
        drills: List[VerticalDrillOperation],
        wp: WorkpieceSpec,
    ) -> List[tuple]:
        """Collapse equally-spaced collinear drills into (dr, xa, ya, count, step) tuples."""
        if not drills:
            return []

        TOL = 0.01
        pts = sorted(
            [(round(wp.width_x - dr.x, 4), round(wp.width_y - dr.y, 4), dr)
             for dr in drills],
            key=lambda t: (round(t[1], 3), round(t[0], 3)),
        )

        result: List[tuple] = []
        i = 0
        while i < len(pts):
            xa0, ya0, dr0 = pts[i]

            row = [(xa0, dr0)]
            j = i + 1
            while j < len(pts) and abs(pts[j][1] - ya0) < TOL:
                row.append((pts[j][0], pts[j][2]))
                j += 1

            row.sort(key=lambda t: t[0])

            grouped = False
            if len(row) > 1:
                step = round(row[1][0] - row[0][0], 4)
                same = all(
                    abs(r[1].depth    - dr0.depth)    < TOL
                    and abs(r[1].diameter - dr0.diameter) < TOL
                    and r[1].tool_number == dr0.tool_number
                    for r in row[1:]
                )
                uniform = all(
                    abs((row[k][0] - row[k - 1][0]) - step) < TOL
                    for k in range(1, len(row))
                )
                if same and uniform and step > TOL:
                    result.append((row[0][1], row[0][0], ya0, len(row), step))
                    i = j
                    grouped = True

            if not grouped:
                result.append((dr0, xa0, ya0, 1, 0.0))
                i += 1

        return result

    def _vertical_drill_block(
        self,
        dr: VerticalDrillOperation,
        ori: int,
        wp: WorkpieceSpec,
        xa: float | None = None,
        ya: float | None = None,
        count: int = 1,
        step: float = 0.0,
    ) -> str:
        """
        Single or repeated vertical drill hole (WoodWOP 9 CadCamLT format).

        S_ is the drill aggregate/bank index (always "2" for the standard
        vertical drill unit), NOT the tool magazine slot.  Tool selection
        is done by diameter (DU) internally by WoodWOP.
        """
        if xa is None:
            xa = wp.width_x - dr.x
        if ya is None:
            ya = wp.width_y - dr.y
        ab = _fmm(step) if count > 1 else "0"
        return (
            f"<102 \\BohrVert\\\n"
            f'XA="{_fmm(xa)}"\n'
            f'YA="{_fmm(ya)}"\n'
            f'BM="LS"\n'
            f'TI="{_fmm(dr.depth)}"\n'
            f'DU="{_fmm(dr.diameter)}"\n'
            f'AN="{count}"\n'
            f'MI="0"\n'
            f'S_="2"\n'
            f'S_P="100"\n'
            f'AB="{ab}"\n'
            f'WI="0"\n'
            f'ZT="0"\n'
            f'RM="0"\n'
            f'VW="0"\n'
            f'HP="0"\n'
            f'SP="0"\n'
            f'YVE="0"\n'
            f'WW="60,61,62,88,90,91,92,150"\n'
            f'ASG="2"\n'
            f'HP_A_O="STANDARD"\n'
            f'KAT="Bohren vertikal"\n'
            f'MNM="Bohren vertikal"\n'
            f'ORI="{ori}"\n'
            f'MX="0"\n'
            f'MY="0"\n'
            f'MZ="0"\n'
            f'MXF="1"\n'
            f'MYF="1"\n'
            f'MZF="1"\n'
            f'SYA="0"\n'
            f'SYV="0"\n'
            f'KO="00"\n'
            f"\n"
        )

    def _freeform_pocket_block(self, pk: PocketOperation, ori: int) -> str:
        n = pk.contour_id
        return (
            f"<181 \\FreiFormTasche\\\n"
            f'EA="{n}:0"\n'
            f'AD="0"\n'
            f'AZ="1"\n'
            f'UZU="0"\n'
            f'ZU="0.5"\n'
            f'TI="{_fmm(pk.depth_mm)}"\n'
            f'ZT="0"\n'
            f'HU="0"\n'
            f'UXY="1"\n'
            f'XY="50"\n'
            f'T_="{pk.tool_number}"\n'
            f'F_="STANDARD"\n'
            f'DS="1"\n'
            f'OSZI="0"\n'
            f'BL="0"\n'
            f'OSZVS="0"\n'
            f'SM="0"\n'
            f'S_="STANDARD"\n'
            f'HP="0"\n'
            f'SP="0"\n'
            f'YVE="0"\n'
            f'WW="{pk.workstations}"\n'
            f'ASG="2"\n'
            f'HP_A_O="STANDARD"\n'
            f'KG="0"\n'
            f'RP="STANDARD"\n'
            f'KAT="Freiformtasche"\n'
            f'MNM="Freiformtasche"\n'
            f'ORI="{ori}"\n'
            f'MX="0"\n'
            f'MY="0"\n'
            f'MZ="0"\n'
            f'MXF="1"\n'
            f'MYF="1"\n'
            f'MZF="1"\n'
            f'SYA="0"\n'
            f'SYV="0"\n'
            f"\n"
        )

    def _contour_routing_block(self, ct: ContourOperation, ori: int) -> str:
        n          = ct.contour_id
        last_point = ct.point_count - 1
        za         = _fmm(ct.depth_mm)

        return (
            f"<105 \\Konturfraesen\\\n"
            f'EA="{n}:0"\n'
            f'MDA="SEN"\n'
            f'STUFEN="0"\n'
            f'BL="0"\n'
            f'WZS="1"\n'
            f'OSZI="0"\n'
            f'OSZVS="0"\n'
            f'ZSTART="0"\n'
            f'ANZZST="0"\n'
            f'RK="{ct.rk}"\n'
            f'EE="{n}:{last_point}"\n'
            f'MDE="SEN_AB"\n'
            f'EM="0"\n'
            f'RI="1"\n'
            f'TNO="{ct.tool_number}"\n'
            f'SM="0"\n'
            f'S_="STANDARD"\n'
            f'F_="STANDARD"\n'
            f'AB="0"\n'
            f'AF="0"\n'
            f'AW="0"\n'
            f'BW="0"\n'
            f'VLS="0"\n'
            f'VLE="0"\n'
            f'ZA="{za}"\n'
            f'SC="0"\n'
            f'TDM="0"\n'
            f'HP="0"\n'
            f'SP="0"\n'
            f'YVE="0"\n'
            f'WW="{ct.workstations}"\n'
            f'ASG="2"\n'
            f'HP_A_O="STANDARD"\n'
            f'KG="0"\n'
            f'RP="STANDARD"\n'
            f'RSEL="0"\n'
            f'RWID="0"\n'
            f'KAT="Fr\xe4sen"\n'
            f'MNM="Fr\xe4sen"\n'
            f'ORI="{ori}"\n'
            f'MX="0"\n'
            f'MY="0"\n'
            f'MZ="0"\n'
            f'MXF="1"\n'
            f'MYF="1"\n'
            f'MZF="1"\n'
            f'SYA="0"\n'
            f'SYV="0"\n'
            f"\n"
        )