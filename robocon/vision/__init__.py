import importlib.util

has_picamera2 = importlib.util.find_spec("picamera2") is not None

if not has_picamera2:
    import sys
    import robocon.vision.mock_picamera2 as mock_picamera2
    sys.modules["picamera2"] = mock_picamera2

import sys

from robocon.vision.camera import Camera, NoCameraPresent
from robocon.vision.vision import RoboConUSBCamera, PhyCamera

MINIUM_VERSION = (3, 6)
if sys.version_info <= MINIUM_VERSION:
    raise ImportError(
        "Expected python {} but instead got {}".format(MINIUM_VERSION, sys.version_info)
    )

__all__ = (
    "Camera",
    "NoCameraPresent",
    "RoboConUSBCamera",
    "PhyCamera",
)
