"""
Shared pytest fixtures.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# Ensure C:\CNC is on the path for all tests
_ROOT = Path(__file__).parent.parent.resolve()
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.config_manager import reset_config, load_config

@pytest.fixture(autouse=True)
def reset_cfg():
    """Reset the config singleton before each test."""
    reset_config()
    yield
    reset_config()


@pytest.fixture
def cfg():
    return load_config()


@pytest.fixture
def sample_dxf_path() -> Path:
    """Path to the first normalised sample DXF."""
    p = _ROOT / "to_woodwop" / "1-VGS-01-SHEET X 1.dxf"
    if not p.exists():
        pytest.skip(f"Sample DXF not found: {p}")
    return p


@pytest.fixture
def sample_dxf_path2() -> Path:
    """Path to the second normalised sample DXF (has drills + pockets)."""
    p = _ROOT / "to_woodwop" / "4-VGS-01-SHEET X 1.dxf"
    if not p.exists():
        pytest.skip(f"Sample DXF not found: {p}")
    return p
