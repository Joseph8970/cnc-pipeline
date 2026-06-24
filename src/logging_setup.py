"""
Logging configuration for the CNC pipeline.

Three rotating log files, each with a distinct purpose:
  system.log    – process lifecycle events (start, stop, file discovery)
  conversion.log – per-file conversion results (success / skip / warnings)
  error.log     – exceptions and hard failures (always written even when
                  the other loggers are at INFO)

All three also echo to the console (stdout) at INFO level.
"""

from __future__ import annotations

import logging
import logging.handlers
import sys
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Logger names used across the codebase
# ---------------------------------------------------------------------------
SYSTEM_LOGGER     = "cnc.system"
CONVERSION_LOGGER = "cnc.conversion"
ERROR_LOGGER      = "cnc.error"

_LOG_FORMAT  = "%(asctime)s [%(levelname)-8s] %(name)s – %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"
_MAX_BYTES   = 5 * 1024 * 1024   # 5 MB per file
_BACKUP_COUNT = 5                 # keep 5 rotated backups


def setup_logging(log_dir: Path, level: int = logging.INFO) -> None:
    """
    Initialise all three loggers.  Safe to call multiple times; subsequent
    calls are no-ops unless handlers were removed.
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_DATE_FORMAT)

    _configure_logger(SYSTEM_LOGGER,     log_dir / "system.log",     formatter, level)
    _configure_logger(CONVERSION_LOGGER, log_dir / "conversion.log", formatter, level)
    _configure_logger(ERROR_LOGGER,      log_dir / "error.log",      formatter, logging.WARNING)


def _configure_logger(
    name: str,
    log_path: Path,
    formatter: logging.Formatter,
    level: int,
) -> None:
    logger = logging.getLogger(name)
    if logger.handlers:
        return  # already configured

    logger.setLevel(level)
    logger.propagate = False

    # Rotating file handler
    fh = logging.handlers.RotatingFileHandler(
        log_path,
        maxBytes=_MAX_BYTES,
        backupCount=_BACKUP_COUNT,
        encoding="utf-8",
    )
    fh.setFormatter(formatter)
    fh.setLevel(level)
    logger.addHandler(fh)

    # Console handler (stdout only, INFO and above)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(formatter)
    ch.setLevel(logging.INFO)
    logger.addHandler(ch)


# ---------------------------------------------------------------------------
# Convenience accessors
# ---------------------------------------------------------------------------

def get_system_logger() -> logging.Logger:
    return logging.getLogger(SYSTEM_LOGGER)


def get_conversion_logger() -> logging.Logger:
    return logging.getLogger(CONVERSION_LOGGER)


def get_error_logger() -> logging.Logger:
    return logging.getLogger(ERROR_LOGGER)
