"""Safety layer package: the guard, plus check helpers."""
from __future__ import annotations

from .guard import (CheckResult, GuardResult, SafetyGuard,
                    STANDOFF_MARGIN, KEEPOUT_MARGIN, FRAME_MAX_AGE)

__all__ = ["CheckResult", "GuardResult", "SafetyGuard",
           "STANDOFF_MARGIN", "KEEPOUT_MARGIN", "FRAME_MAX_AGE"]
