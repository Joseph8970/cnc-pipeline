#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF Watcher -- production version.

Watches C:\\CNC\\inbox_raw_dxf (recursively) for new DXF files.

NON-PLYWOOD workflow per file:
  1. dxf_normalizer.py  ->  to_woodwop/
  2. dxf_to_mpr.py      ->  mpr_out/
  3. Archive raw DXF

PLYWOOD workflow per file:
  1. dxf_normalizer.py     ->  to_woodwop/
  2. auto_concave_holes.py ->  to_woodwop2/
  3. dxf_to_mpr.py         ->  mpr_out/     (from the to_woodwop2 copy)
  4. Archive raw DXF

Material is determined from the subfolder name under inbox_raw_dxf,
using the same _material_from_folder() logic as dxf_normalizer.py.

Single-instance enforced via a file lock (Windows msvcrt).
Per-write de-dupe: skips re-processing identical (mtime, size) files.
"""

from __future__ import annotations

import hashlib
import os
import re
import shutil
import subprocess
import sys
import time
import queue
import threading
from datetime import datetime
from pathlib import Path

# Ensure src/ and project root are importable
_HERE = Path(__file__).parent.resolve()
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from src.config_manager import get_config
from src.logging_setup import setup_logging, get_system_logger, get_conversion_logger, get_error_logger

# Import material-detection logic from the normalizer (read-only use)
from dxf_normalizer import _material_from_folder

# ---------------------------------------------------------------------------
# Config and logging
# ---------------------------------------------------------------------------
_cfg  = get_config()
setup_logging(_cfg.paths.logs)
_slog = get_system_logger()
_clog = get_conversion_logger()
_elog = get_error_logger()

# Folder that receives plywood DXFs after concave-hole processing
_WOODWOP2 = _HERE / "to_woodwop2"

# Parse normalizer stdout: "[OK] bin_001.dxf -> C:\CNC\to_woodwop\1-PLYWOOD-SHEET X 1.dxf"
_OK_RE = re.compile(r"\[OK\]\s+\S.*?->\s+(.+)")

# ---------------------------------------------------------------------------
# Single-instance lock (Windows)
# ---------------------------------------------------------------------------
_LOCKFILE = _HERE / "watch_dxf.lock"
_lock_fh  = None

try:
    import msvcrt

    def acquire_single_instance_lock() -> bool:
        global _lock_fh
        _lock_fh = open(_LOCKFILE, "w")
        try:
            msvcrt.locking(_lock_fh.fileno(), msvcrt.LK_NBLCK, 1)
            _lock_fh.write(str(os.getpid()))
            _lock_fh.flush()
            return True
        except OSError:
            return False

except ImportError:
    def acquire_single_instance_lock() -> bool:
        return True


# ---------------------------------------------------------------------------
# De-dupe guard: path -> (mtime, size)
# ---------------------------------------------------------------------------
_PROCESSED: dict[str, tuple[float, int]] = {}
_GUARD_LOCK = threading.Lock()


def _is_new_write(p: Path) -> bool:
    try:
        st = p.stat()
    except FileNotFoundError:
        return False
    key = str(p.resolve())
    sig = (st.st_mtime, st.st_size)
    with _GUARD_LOCK:
        if _PROCESSED.get(key) == sig:
            return False
        _PROCESSED[key] = sig
    return True


# ---------------------------------------------------------------------------
# Material helpers
# ---------------------------------------------------------------------------

def _detect_material(raw_dxf: Path) -> str:
    """Return the material token from the subfolder name under inbox_raw_dxf.
    Falls back to 'UNKNOWN' if path layout doesn't match expected structure."""
    mat = _material_from_folder(raw_dxf)
    return mat.upper() if mat else "UNKNOWN"


def _is_plywood(material: str) -> bool:
    return material.upper().startswith("PLYWOOD")


# ---------------------------------------------------------------------------
# Normalizer output discovery
# ---------------------------------------------------------------------------

def _parse_normalizer_output(stdout: str, out_dir: Path) -> list[Path]:
    """Extract output file paths from normalizer stdout lines like:
      [OK] bin_001.dxf -> C:\\CNC\\to_woodwop\\1-PLYWOOD-SHEET X 1.dxf
    """
    paths: list[Path] = []
    for line in stdout.splitlines():
        m = _OK_RE.search(line.strip())
        if m:
            p = Path(m.group(1).strip())
            # Make absolute relative to out_dir if the path isn't already
            if not p.is_absolute():
                p = out_dir / p
            if p.exists():
                paths.append(p)
    return paths


# ---------------------------------------------------------------------------
# Diagnostic helpers
# ---------------------------------------------------------------------------

def _file_fingerprint(p: Path) -> dict:
    """Return size, mtime, and MD5 for *p*."""
    try:
        st   = p.stat()
        md5  = hashlib.md5(p.read_bytes()).hexdigest()
        return {
            "size":  st.st_size,
            "mtime": datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
            "md5":   md5,
        }
    except Exception as exc:
        return {"size": "?", "mtime": "?", "md5": f"ERROR: {exc}"}


def _diag(label: str, p: Path | None, *, fingerprint: bool = False) -> None:
    """Print a single diagnostic line to stdout (always) and the conversion log."""
    if p is None:
        line = f"[DIAG] {label}: <none>"
    else:
        line = f"[DIAG] {label}: {p.resolve()}"
        if fingerprint and p.exists():
            fp = _file_fingerprint(p)
            line += (
                f"\n       size  = {fp['size']} bytes"
                f"\n       mtime = {fp['mtime']}"
                f"\n       md5   = {fp['md5']}"
            )
    print(line, flush=True)
    _clog.info(line)


# ---------------------------------------------------------------------------
# Pipeline steps
# ---------------------------------------------------------------------------

def _run_normalizer(raw_dxf: Path, out_dir: Path) -> list[Path]:
    """Run dxf_normalizer.py on *raw_dxf*, output to *out_dir*.
    Returns list of produced normalized DXF paths."""
    normaliser = _cfg.paths.normalizer
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        result = subprocess.run(
            [sys.executable, str(normaliser), str(raw_dxf), "-o", str(out_dir)],
            capture_output=True,
            text=True,
            timeout=120,
        )
    except subprocess.TimeoutExpired:
        _elog.error("Normalizer timed out for %s", raw_dxf.name)
        return []
    except Exception as exc:
        _elog.exception("Normalizer failed for %s: %s", raw_dxf.name, exc)
        return []

    # Log normalizer output at debug level
    for line in result.stdout.splitlines():
        _clog.debug("  [normalizer] %s", line)
    if result.stderr:
        for line in result.stderr.splitlines():
            _elog.error("  [normalizer] %s", line)

    if result.returncode != 0:
        _elog.error("Normalizer exited %d for %s", result.returncode, raw_dxf.name)
        return []

    produced = _parse_normalizer_output(result.stdout, out_dir)
    if not produced:
        _elog.error("Normalizer produced no output for %s", raw_dxf.name)
    return produced


def _run_concave_holes(norm_dxf: Path, out_dir: Path) -> Path | None:
    """Run auto_concave_holes.py as a subprocess on *norm_dxf*, saving to *out_dir*.

    Running as a subprocess (not an import) guarantees the OS has fully flushed
    ezdxf's file writes to disk before dxf_to_mpr reads the output.
    Returns the output path, or None on failure.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / norm_dxf.name
    script   = _HERE / "auto_concave_holes.py"
    try:
        result = subprocess.run(
            [sys.executable, str(script), str(norm_dxf), str(out_path)],
            capture_output=True, text=True, timeout=120,
            cwd=str(_HERE),
        )
    except subprocess.TimeoutExpired:
        _elog.error("auto_concave_holes timed out for %s", norm_dxf.name)
        return None
    except Exception as exc:
        _elog.exception("auto_concave_holes failed for %s: %s", norm_dxf.name, exc)
        return None

    for line in result.stdout.splitlines():
        _clog.debug("  [concave] %s", line)
    if result.stderr:
        for line in result.stderr.splitlines():
            _elog.error("  [concave stderr] %s", line)

    if result.returncode != 0:
        _elog.error("auto_concave_holes exited %d for %s", result.returncode, norm_dxf.name)
        return None

    if not out_path.exists():
        _elog.error("auto_concave_holes produced no output file for %s", norm_dxf.name)
        return None

    _clog.info("  Concave holes done -> %s", out_path.name)
    return out_path


def _run_mpr(norm_dxf: Path) -> int:
    """Convert one normalized DXF to MPR via subprocess. Returns 1 on success, 0 on failure.

    Running as a subprocess (not an import) guarantees the latest code is used
    on every conversion without needing to restart the watcher.
    """
    dxf_to_mpr_path = _HERE / "dxf_to_mpr.py"
    try:
        result = subprocess.run(
            [sys.executable, str(dxf_to_mpr_path), str(norm_dxf)],
            capture_output=True, text=True, timeout=120,
            cwd=str(_HERE),
        )
    except subprocess.TimeoutExpired:
        _elog.error("dxf_to_mpr timed out for %s", norm_dxf.name)
        return 0
    except Exception as exc:
        _elog.exception("dxf_to_mpr failed for %s: %s", norm_dxf.name, exc)
        return 0

    for line in result.stdout.splitlines():
        _clog.debug("  [mpr] %s", line)
    if result.stderr:
        for line in result.stderr.splitlines():
            _elog.error("  [mpr stderr] %s", line)

    if result.returncode != 0:
        _elog.error("dxf_to_mpr exited %d for %s", result.returncode, norm_dxf.name)
        return 0

    return 1


# ---------------------------------------------------------------------------
# Main per-file processor
# ---------------------------------------------------------------------------

def _process(raw_dxf: Path) -> None:
    """Full pipeline for one raw DXF file."""

    # ---- Detect material --------------------------------------------------
    material = _detect_material(raw_dxf)
    plywood  = _is_plywood(material)

    _slog.info("Processing: %s", raw_dxf.name)
    _clog.info("Detected Material: %s", material)
    _diag("Raw DXF", raw_dxf)

    # ---- 1. Normalize -> to_woodwop/ -------------------------------------
    _clog.info("Running Normalizer...")
    to_woodwop  = _cfg.paths.to_woodwop
    norm_files  = _run_normalizer(raw_dxf, to_woodwop)
    if not norm_files:
        return

    _clog.info("Normalized %d file(s): %s",
               len(norm_files), ", ".join(f.name for f in norm_files))
    for nf in norm_files:
        _diag("Normalized DXF", nf)

    # ---- 2. (PLYWOOD only) Concave holes -> to_woodwop2/ -----------------
    # Plywood workflow: normalizer -> auto_concave_holes -> dxf_to_mpr
    # Non-plywood workflow: normalizer -> dxf_to_mpr  (skip concave holes)
    if plywood:
        _clog.info("Running Concave Hole Processor (plywood path)...")
        processed: list[Path] = []
        for nf in norm_files:
            out = _run_concave_holes(nf, _WOODWOP2)
            if out:
                _diag("Concave-hole DXF", out, fingerprint=True)
                processed.append(out)
            else:
                _elog.warning("Concave hole processor produced no output for %s", nf.name)
        if not processed:
            _elog.error("No concave-hole output — cannot generate MPR for %s", raw_dxf.name)
            return
        mpr_sources = processed   # use to_woodwop2/ output for plywood MPR
    else:
        _diag("Concave-hole DXF", None)   # step skipped for non-plywood
        mpr_sources = norm_files           # use to_woodwop/ output directly

    # ---- 3. Generate MPR -------------------------------------------------
    _clog.info("Generating MPR...")
    total_mpr = 0
    for src in mpr_sources:
        _diag("Final DXF -> ConversionPipeline", src, fingerprint=True)
        expected_mpr = _cfg.paths.mpr_out / (src.stem + ".mpr")
        total_mpr += _run_mpr(src)
        _diag("Final MPR", expected_mpr if expected_mpr.exists() else None)

    _clog.info("%s -> %d MPR file(s) written", raw_dxf.name, total_mpr)

    # ---- 4. Archive raw DXF ----------------------------------------------
    archive_dir = _cfg.paths.archive
    archive_dir.mkdir(parents=True, exist_ok=True)

    # Preserve relative subfolder to avoid name collisions across materials
    try:
        rel = raw_dxf.resolve().relative_to(_cfg.paths.inbox_raw_dxf.resolve())
        dest = archive_dir / rel
    except ValueError:
        dest = archive_dir / raw_dxf.name

    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        ts   = time.strftime("%Y%m%d_%H%M%S")
        dest = dest.with_name(f"{dest.stem}_{ts}{dest.suffix}")

    try:
        shutil.move(str(raw_dxf), str(dest))
        _slog.info("Archived: %s -> %s", raw_dxf.name, dest)
    except Exception as exc:
        _elog.warning("Could not archive %s: %s", raw_dxf.name, exc)


# ---------------------------------------------------------------------------
# Worker thread
# ---------------------------------------------------------------------------

class _Worker(threading.Thread):
    def __init__(self, work_queue: "queue.Queue[Path]") -> None:
        super().__init__(daemon=True, name="DXFWorker")
        self._q = work_queue

    def run(self) -> None:
        while True:
            path = self._q.get()
            try:
                _process(path)
            except Exception as exc:
                _elog.exception("Unhandled error processing %s: %s", path.name, exc)
            finally:
                self._q.task_done()


# ---------------------------------------------------------------------------
# Watchdog event handler
# ---------------------------------------------------------------------------

class _DXFHandler(FileSystemEventHandler):
    _IGNORE_SUFFIXES = ("~", ".tmp", ".part", ".crdownload")

    def __init__(self, work_queue: "queue.Queue[Path]", debounce: float) -> None:
        self._q        = work_queue
        self._debounce = debounce
        self._pending: dict[str, threading.Timer] = {}
        self._lock     = threading.Lock()

    def on_created(self, event) -> None:
        self._enqueue(event.src_path, event.is_directory)

    def on_moved(self, event) -> None:
        # Atomic saves: editor writes a temp file then renames it
        self._enqueue(event.dest_path, event.is_directory)

    def _enqueue(self, src_path: str, is_dir: bool) -> None:
        if is_dir:
            return
        p = Path(src_path)
        if not self._is_valid(p):
            return
        key = str(p.resolve())
        with self._lock:
            if key in self._pending:
                self._pending[key].cancel()
            t = threading.Timer(self._debounce, self._fire, args=(p,))
            self._pending[key] = t
            t.start()

    def _fire(self, p: Path) -> None:
        with self._lock:
            self._pending.pop(str(p.resolve()), None)
        if not p.exists():
            return
        if not _is_new_write(p):
            return
        self._q.put(p)

    def _is_valid(self, p: Path) -> bool:
        if p.suffix.lower() not in _cfg.watcher.extensions:
            return False
        name = p.name.lower()
        if any(name.endswith(s) for s in self._IGNORE_SUFFIXES) or name.startswith("~$"):
            return False
        try:
            resolved = p.resolve()
            # Never process files that live in output or archive folders
            for skip in (_cfg.paths.to_woodwop, _cfg.paths.mpr_out,
                         _cfg.paths.archive, _WOODWOP2):
                if resolved.is_relative_to(skip.resolve()):
                    return False
        except Exception:
            pass
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    if not acquire_single_instance_lock():
        print("Another watcher instance is already running.")
        sys.exit(1)

    inbox = _cfg.paths.inbox_raw_dxf
    inbox.mkdir(parents=True, exist_ok=True)

    work_queue: "queue.Queue[Path]" = queue.Queue()
    worker = _Worker(work_queue)
    worker.start()

    handler  = _DXFHandler(work_queue, _cfg.watcher.debounce_seconds)
    observer = Observer()
    observer.schedule(handler, str(inbox), recursive=True)   # recursive!
    observer.start()

    _slog.info("Watching (recursive): %s", inbox)
    _slog.info("MPR output: %s", _cfg.paths.mpr_out)
    _slog.info("Press Ctrl+C to stop.")

    if _cfg.watcher.process_existing_on_start:
        for f in sorted(inbox.rglob("*.dxf")):   # recursive!
            if _is_new_write(f):
                work_queue.put(f)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        _slog.info("Shutting down.")
    finally:
        observer.stop()
        observer.join()
        work_queue.join()


if __name__ == "__main__":
    main()
