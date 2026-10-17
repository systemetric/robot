from .teams import TEAM
from .markers import (
    MARKER,
    MARKER_TYPE,
    ARENA_MARKER,
    BASE_MARKER,
    POWER_STATION_MARKER,
    REFUEL_STATION_MARKER
)
from .startup_poems import POEM_ON_STARTUP
from .targets import TARGET_TYPE

PURPLE = (255, 0, 215)  # Purple
ORANGE = (0, 128, 255)  # Orange
YELLOW = (0, 255, 255)  # Yellow
GREEN = (0, 255, 0)  # Green
RED = (0, 0, 255)  # Red
BLUE = (255, 0, 0)  # Blue
WHITE = (255, 255, 255)  # White

__all__ = (
    "TEAM",
    "MARKER",
    "MARKER_TYPE",
    "BASE_MARKER",
    "TARGET_TYPE",
    "ARENA_MARKER",
    "POEM_ON_STARTUP",
    "POWER_STATION_MARKER",
    "REFUEL_STATION_MARKER"
)
