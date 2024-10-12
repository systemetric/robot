from .teams import TEAM
from .markers import (
    MARKER,
    MARKER_TYPE,
    ARENA_MARKER,
    TARGET_MARKER,
    BASE_MARKER,
)
from .startup_poems import POEM_ON_STARTUP

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
    "TARGET_MARKER",
    "MARKER_TYPE",
    "BASE_MARKER",
    "ARENA_MARKER",
    "POEM_ON_STARTUP",
)
