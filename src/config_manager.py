"""
Configuration manager.

Loads config/config.json (or a caller-supplied path) and exposes every
section as typed attributes.  All callers import a single shared instance
via ``get_config()``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Dataclasses that mirror config.json structure
# ---------------------------------------------------------------------------

@dataclass
class PathsConfig:
    inbox_raw_dxf: Path
    to_woodwop:    Path
    mpr_out:       Path
    archive:       Path
    logs:          Path
    normalizer:    Path


@dataclass
class MachiningConfig:
    through_tolerance_mm:  float = 0.5
    spoilboard_overrun_mm: float = 1.0
    contour_z_approach_mm: float = 3.0
    default_feed_rate:     float = 10.0
    default_spindle_rpm:   int   = 18000


@dataclass
class DrillToolConfig:
    diameters:    List[float]
    tool_numbers: Dict[str, str]


@dataclass
class SimpleToolConfig:
    default_tool_number: str
    default_diameter:    float
    workstations:        str = ""


@dataclass
class RoutingToolsConfig:
    """Tool numbers and parameters for contour routing operations."""
    outer_scoring_tno:       str   = "134"
    outer_through_tno:       str   = "130"
    pocket_tno:              str   = "132"
    outer_scoring_depth:     float = 2.0
    workstations:            str   = "1,2,3,401,402,403"
    # Tool 130 finish-pass parameters
    plywood_finish_depth:    float = 0.7   # depth for T130 pass on PLYWOOD parts
    small_part_threshold_mm: float = 76.2  # 3 inches — boundary between small and large
    small_part_finish_depth: float = 0.5   # T130 depth when min(w,h) <= threshold
    large_part_finish_depth: float = 0.0   # T130 depth when min(w,h) > threshold
    pocket_workstations:     str   = "1,3,4,133,135,137,139,211,213,214,215,216,217,401,403"


@dataclass
class ToolsConfig:
    vertical_drill:    DrillToolConfig
    horizontal_drill:  SimpleToolConfig
    contour_routing:   SimpleToolConfig
    groove:            SimpleToolConfig
    routing_tools:     RoutingToolsConfig = field(default_factory=RoutingToolsConfig)


@dataclass
class LayerMappingConfig:
    workpiece_prefix:         str = "ProcPart_"
    outer_contour_layer:      str = "V_Fraes_2T134R"
    drill_prefix:             str = "V_DrillSF_"
    routing_prefix:           str = "V_Fraes_"
    right_compensation_suffix: str = "T134R"
    left_compensation_suffix:  str = "T134L"


@dataclass
class MprConfig:
    version:         str   = "4.0 Alpha"
    woodwop_version: str   = "9.0.152"
    profile:         str   = "CadCamLT"
    material:        str   = "HOMAG"
    default_fnx:     float = 0.0
    default_fny:     float = 0.0


@dataclass
class OutputConfig:
    overwrite_existing: bool = True
    filename_template:  str  = "{label}_{block}.mpr"
    unknown_label:      str  = "PART"


@dataclass
class WatcherConfig:
    debounce_seconds:          float      = 1.5
    process_existing_on_start: bool       = False
    extensions:                List[str]  = field(default_factory=lambda: [".dxf"])


@dataclass
class Config:
    paths:         PathsConfig
    machining:     MachiningConfig
    tools:         ToolsConfig
    layer_mapping: LayerMappingConfig
    mpr:           MprConfig
    output:        OutputConfig
    watcher:       WatcherConfig

    def tool_number_for_diameter(self, diameter_mm: float) -> str:
        key = f"{diameter_mm:.1f}".rstrip("0").rstrip(".")
        numbers = self.tools.vertical_drill.tool_numbers
        if key in numbers:
            return numbers[key]
        for k, v in numbers.items():
            if k == "default":
                continue
            try:
                if abs(float(k) - diameter_mm) < 0.1:
                    return v
            except ValueError:
                pass
        return numbers.get("default", "60")


# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / "config" / "config.json"
_cached_config: Optional[Config] = None


def load_config(path: Optional[Path] = None) -> Config:
    global _cached_config

    config_path = Path(path) if path else _DEFAULT_CONFIG_PATH
    raw = json.loads(config_path.read_text(encoding="utf-8-sig"))

    p = raw["paths"]
    paths = PathsConfig(
        inbox_raw_dxf=Path(p["inbox_raw_dxf"]),
        to_woodwop=Path(p["to_woodwop"]),
        mpr_out=Path(p["mpr_out"]),
        archive=Path(p["archive"]),
        logs=Path(p["logs"]),
        normalizer=Path(p["normalizer"]),
    )

    m = raw.get("machining", {})
    machining = MachiningConfig(
        through_tolerance_mm=m.get("through_tolerance_mm", 0.5),
        spoilboard_overrun_mm=m.get("spoilboard_overrun_mm", 1.0),
        contour_z_approach_mm=m.get("contour_z_approach_mm", 3.0),
        default_feed_rate=m.get("default_feed_rate", 10.0),
        default_spindle_rpm=int(m.get("default_spindle_rpm", 18000)),
    )

    t = raw["tools"]
    vd = t["vertical_drill"]
    rt = t.get("routing_tools", {})
    tools = ToolsConfig(
        vertical_drill=DrillToolConfig(
            diameters=vd["diameters"],
            tool_numbers=vd["tool_numbers"],
        ),
        horizontal_drill=SimpleToolConfig(
            default_tool_number=t["horizontal_drill"]["default_tool_number"],
            default_diameter=t["horizontal_drill"]["default_diameter"],
        ),
        contour_routing=SimpleToolConfig(
            default_tool_number=t["contour_routing"]["default_tool_number"],
            default_diameter=t["contour_routing"]["default_diameter"],
            workstations=t["contour_routing"].get("workstations", ""),
        ),
        groove=SimpleToolConfig(
            default_tool_number=t["groove"]["default_tool_number"],
            default_diameter=t["groove"]["default_diameter"],
            workstations=t["groove"].get("workstations", ""),
        ),
        routing_tools=RoutingToolsConfig(
            outer_scoring_tno=rt.get("outer_scoring_tno", "134"),
            outer_through_tno=rt.get("outer_through_tno", "130"),
            pocket_tno=rt.get("pocket_tno", "132"),
            outer_scoring_depth=float(rt.get("outer_scoring_depth", 2.0)),
            workstations=rt.get("workstations", "1,2,3,401,402,403"),
            plywood_finish_depth=float(rt.get("plywood_finish_depth", 0.7)),
            small_part_threshold_mm=float(rt.get("small_part_threshold_mm", 76.2)),
            small_part_finish_depth=float(rt.get("small_part_finish_depth", 0.5)),
            large_part_finish_depth=float(rt.get("large_part_finish_depth", 0.0)),
            pocket_workstations=rt.get(
                "pocket_workstations",
                "1,3,4,133,135,137,139,211,213,214,215,216,217,401,403",
            ),
        ),
    )

    lm = raw.get("layer_mapping", {})
    layer_mapping = LayerMappingConfig(
        workpiece_prefix=lm.get("workpiece_prefix", "ProcPart_"),
        outer_contour_layer=lm.get("outer_contour_layer", "V_Fraes_2T134R"),
        drill_prefix=lm.get("drill_prefix", "V_DrillSF_"),
        routing_prefix=lm.get("routing_prefix", "V_Fraes_"),
        right_compensation_suffix=lm.get("right_compensation_suffix", "T134R"),
        left_compensation_suffix=lm.get("left_compensation_suffix", "T134L"),
    )

    mr = raw.get("mpr", {})
    mpr = MprConfig(
        version=mr.get("version", "4.0 Alpha"),
        woodwop_version=mr.get("woodwop_version", "9.0.152"),
        profile=mr.get("profile", "CadCamLT"),
        material=mr.get("material", "HOMAG"),
        default_fnx=mr.get("default_fnx", 0.0),
        default_fny=mr.get("default_fny", 0.0),
    )

    o = raw.get("output", {})
    output = OutputConfig(
        overwrite_existing=o.get("overwrite_existing", True),
        filename_template=o.get("filename_template", "{label}_{block}.mpr"),
        unknown_label=o.get("unknown_label", "PART"),
    )

    w = raw.get("watcher", {})
    watcher = WatcherConfig(
        debounce_seconds=w.get("debounce_seconds", 1.5),
        process_existing_on_start=w.get("process_existing_on_start", False),
        extensions=w.get("extensions", [".dxf"]),
    )

    _cached_config = Config(
        paths=paths,
        machining=machining,
        tools=tools,
        layer_mapping=layer_mapping,
        mpr=mpr,
        output=output,
        watcher=watcher,
    )
    return _cached_config


def get_config(path: Optional[Path] = None) -> Config:
    global _cached_config
    if _cached_config is None:
        return load_config(path)
    return _cached_config


def reset_config() -> None:
    global _cached_config
    _cached_config = None
