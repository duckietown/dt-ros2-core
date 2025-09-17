"""Lane filter package initialization with compatibility helpers."""

from collections import namedtuple
import importlib
import inspect
from typing import Any

# Provide backwards-compatible inspect.ArgSpec removed in Python 3.12.
if not hasattr(inspect, "ArgSpec"):
    inspect.ArgSpec = namedtuple("ArgSpec", "args varargs keywords defaults")

FAMILY_LANE_FILTER = "lane_filter"

__all__ = ["LaneFilterHistogram", "LaneFilterClassic", "FAMILY_LANE_FILTER"]


def __getattr__(name: str) -> Any:
    if name == "LaneFilterHistogram":
        return importlib.import_module(".lane_filter", __name__).LaneFilterHistogram
    if name == "LaneFilterClassic":
        return importlib.import_module(".lane_filter_classic", __name__).LaneFilterClassic
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
