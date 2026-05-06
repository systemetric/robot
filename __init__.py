from robocon.game import (
    TEAM,
    TARGET_TYPE,
    MARKER,
    TARGET_MARKER,
    MARKER_TYPE,
    BASE_MARKER,
    ARENA_MARKER)

from enum import Enum
class MODE(Enum):
    COMP = "comp"
    DEV = "dev"

zone = TEAM.RED
mode = MODE.COMP

__all__ = (
    "TEAM",
    "TARGET_TYPE",
    "MARKER",
    "TARGET_MARKER",
    "MARKER_TYPE",
    "BASE_MARKER",
    "ARENA_MARKER",
)
