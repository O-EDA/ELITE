"""Python-facing helpers for the in-process ELITE C++ extension."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

try:
    from _elite_cpp import LutOptimizer, route
except ImportError as exc:  # pragma: no cover - exercised only before installation
    raise ImportError(
        "ELITE's native extension is not built. Run `python -m pip install -e .` "
        "from the repository root."
    ) from exc


@lru_cache(maxsize=1)
def lut_optimizer(lut_path: str | Path, max_level: int = 10) -> LutOptimizer:
    """Load and cache the large LUT once per process/configuration."""
    return LutOptimizer(str(Path(lut_path).resolve()), int(max_level))
