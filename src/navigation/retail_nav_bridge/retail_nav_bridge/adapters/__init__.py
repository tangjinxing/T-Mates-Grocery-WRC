"""Adapter package."""

from retail_nav_bridge.adapters.base import (
    AdapterResult,
    HealthInfo,
    NavAdapter,
    NavGoal,
    Pose2D,
)
from retail_nav_bridge.adapters.mock import MockNavAdapter

__all__ = [
    "AdapterResult",
    "HealthInfo",
    "MockNavAdapter",
    "NavAdapter",
    "NavGoal",
    "Pose2D",
    "S2NavAdapter",
    "LhNavAdapter",
    "WooshNavAdapter",
]


def __getattr__(name: str):
    # Lazy import: vendor adapters need ROS vendor msgs; keep mock/pytest importable.
    if name == "S2NavAdapter":
        from retail_nav_bridge.adapters.s2 import S2NavAdapter

        return S2NavAdapter
    if name == "LhNavAdapter":
        from retail_nav_bridge.adapters.lh import LhNavAdapter

        return LhNavAdapter
    if name == "WooshNavAdapter":
        from retail_nav_bridge.adapters.woosh import WooshNavAdapter

        return WooshNavAdapter
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
