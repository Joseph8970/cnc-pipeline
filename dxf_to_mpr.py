#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF → MPR pipeline – CLI entry point.

One MPR file is produced per DXF sheet file.  The MPR workpiece equals the
full sheet dimensions; all part contours, pockets and drills are placed at
their sheet-level world coordinates.

Usage
─────
    # Convert all normalised DXFs in to_woodwop/ → mpr_out/
    python dxf_to_mpr.py

    # Convert a single file
    python dxf_to_mpr.py path/to/file.dxf

    # Normalise + convert a raw DXF in one step
    python dxf_to_mpr.py --raw path/to/raw.dxf

    # Use a non-default config
    python dxf_to_mpr.py --config path/to/config.json
"""

from __future__ import annotations

import argparse
import hashlib
import logging
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

_OK_RE = re.compile(r"\[OK\]\s+\S.*?->\s+(.+)")

_HERE = Path(__file__).parent.resolve()
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from src.config_manager import get_config, load_config
from src.logging_setup import setup_logging, get_system_logger, get_conversion_logger, get_error_logger
from src.parser.dxf_parser import DXFParser
from src.toolpath.toolpath_engine import ToolpathEngine
from src.writer.mpr_writer import MPRWriter
from src.validation.validator import PartValidator


# ---------------------------------------------------------------------------
# Core pipeline
# ---------------------------------------------------------------------------

class ConversionPipeline:
    """
    Converts one normalised DXF sheet to one MPR file.
    Returns the written Path, or None on failure.
    """

    def __init__(self) -> None:
        self._cfg       = get_config()
        self._parser    = DXFParser()
        self._engine    = ToolpathEngine(self._cfg)
        self._validator = PartValidator()
        self._writer    = MPRWriter(
            woodwop_version=self._cfg.mpr.woodwop_version,
            mpr_version=self._cfg.mpr.version,
            material=self._cfg.mpr.material,
            profile=self._cfg.mpr.profile,
            fnx=self._cfg.mpr.default_fnx,
            fny=self._cfg.mpr.default_fny,
        )
        self._sys_log = get_system_logger()
        self._cvt_log = get_conversion_logger()
        self._err_log = get_error_logger()

    # ------------------------------------------------------------------

    def convert_file(self, dxf_path: Path) -> list[Path]:
        """Parse *dxf_path* and write one MPR for the whole sheet."""
        self._sys_log.info("Converting: %s", dxf_path.name)
        t0 = time.perf_counter()

        try:
            sheet = self._parser.parse_file(dxf_path)
        except Exception as exc:
            self._err_log.exception("Parse failed for %s: %s", dxf_path.name, exc)
            return []

        try:
            ops = self._engine.process(sheet)
        except Exception as exc:
            self._err_log.exception("Toolpath failed for %s: %s", dxf_path.name, exc)
            return []

        result = self._validator.validate(ops)
        if not result.is_valid:
            self._err_log.error("Validation failed for %s:\n%s", dxf_path.name, result)
            return []
        for w in result.warnings:
            self._cvt_log.warning("%s: %s", dxf_path.name, w)

        out_path = self._output_path(dxf_path)
        if out_path.exists() and not self._cfg.output.overwrite_existing:
            self._cvt_log.info("Skip (exists): %s", out_path.name)
            return []

        # Diagnostic fingerprint – printed to stdout so both manual and
        # watcher (subprocess) runs produce comparable output.
        try:
            st  = dxf_path.stat()
            md5 = hashlib.md5(dxf_path.read_bytes()).hexdigest()
            print(
                f"[DIAG] Final DXF -> ConversionPipeline: {dxf_path.resolve()}\n"
                f"       size  = {st.st_size} bytes\n"
                f"       mtime = {datetime.fromtimestamp(st.st_mtime).strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"       md5   = {md5}",
                flush=True,
            )
        except Exception:
            pass

        try:
            self._writer.write_file(ops, out_path)
        except Exception as exc:
            self._err_log.exception("Write failed for %s: %s", out_path, exc)
            return []

        print(f"[DIAG] Final MPR: {out_path.resolve()}", flush=True)
        self._cvt_log.info(
            "Written: %s  (%.0fx%.0f mm, %d ops, %.1f ms)",
            out_path.name,
            ops.workpiece.width_x,
            ops.workpiece.width_y,
            ops.operation_count,
            (time.perf_counter() - t0) * 1000,
        )
        return [out_path]

    def convert_directory(self, dxf_dir: Path) -> list[Path]:
        all_written: list[Path] = []
        dxf_files = sorted({f for f in dxf_dir.glob("*") if f.suffix.lower() == ".dxf"})
        if not dxf_files:
            self._cvt_log.warning("No DXF files found in %s", dxf_dir)
            return []
        for f in dxf_files:
            all_written.extend(self.convert_file(f))
        return all_written

    def normalise_and_convert(self, raw_dxf: Path) -> list[Path]:
        normaliser = self._cfg.paths.normalizer
        out_dir    = self._cfg.paths.to_woodwop
        out_dir.mkdir(parents=True, exist_ok=True)
        self._sys_log.info("Normalising: %s", raw_dxf.name)
        try:
            result = subprocess.run(
                [sys.executable, str(normaliser), str(raw_dxf), "-o", str(out_dir)],
                check=True, capture_output=True, text=True,
            )
        except subprocess.CalledProcessError as exc:
            self._err_log.error("Normaliser failed for %s:\n%s", raw_dxf.name, exc.stderr)
            return []

        # Discover output paths from normaliser stdout: "[OK] <name>.dxf -> <abs-path>"
        candidates: list[Path] = []
        for line in result.stdout.splitlines():
            m = _OK_RE.search(line.strip())
            if m:
                p = Path(m.group(1).strip())
                if not p.is_absolute():
                    p = out_dir / p
                if p.exists():
                    candidates.append(p)

        if not candidates:
            self._err_log.error("Normaliser produced no output for %s", raw_dxf.name)
            return []

        all_written: list[Path] = []
        for f in candidates:
            all_written.extend(self.convert_file(f))
        return all_written

    # ------------------------------------------------------------------

    def _output_path(self, dxf_path: Path) -> Path:
        """MPR filename = DXF stem + .mpr"""
        return self._cfg.paths.mpr_out / (dxf_path.stem + ".mpr")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="dxf_to_mpr",
        description="Convert normalised DXF sheets to WoodWOP MPR files (one MPR per sheet).",
    )
    p.add_argument("input", nargs="?",
                   help="DXF file to convert. Omit to convert all files in to_woodwop/.")
    p.add_argument("--raw", action="store_true",
                   help="Treat INPUT as a raw DXF; run the normaliser first.")
    p.add_argument("--config", metavar="PATH",
                   help="Path to config.json (default: config/config.json).")
    p.add_argument("--verbose", "-v", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args     = _build_arg_parser().parse_args(argv)
    cfg_path = Path(args.config) if args.config else None
    cfg      = load_config(cfg_path)

    setup_logging(cfg.paths.logs, level=logging.DEBUG if args.verbose else logging.INFO)
    slog = get_system_logger()
    slog.info("CNC Pipeline started")

    pipeline = ConversionPipeline()

    if args.input:
        dxf_path = Path(args.input)
        if not dxf_path.exists():
            slog.error("File not found: %s", dxf_path)
            return 1
        written = (pipeline.normalise_and_convert(dxf_path)
                   if args.raw else pipeline.convert_file(dxf_path))
    else:
        written = pipeline.convert_directory(cfg.paths.to_woodwop)

    slog.info("Done – %d MPR file(s) written.", len(written))
    return 0


if __name__ == "__main__":
    sys.exit(main())
