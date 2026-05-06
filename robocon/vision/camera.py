"""
The module containing the `Robot` class

Mainly provides init routine for the brain and binds attributes of the `Robot`
class to their respecitve classes
"""
import json
import sys
import optparse
import os
import logging
import time
import threading
import random
import typing

from datetime import datetime
from robocon.game import TEAM, POEM_ON_STARTUP
from . import vision

from hopper import HopperPipe, HopperPipeType, JsonReader

_logger = logging.getLogger("robot_vision")

def setup_logging(level):
    """Display the just the message when logging events
    Sets the logging level to `level`"""
    _logger.setLevel(level)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    fmt = logging.Formatter("%(message)s")
    handler.setFormatter(fmt)

    _logger.addHandler(handler)


class NoCameraPresent(Exception):
    """Camera not connected."""

    def __str__(self):
        return "No camera found."

class Camera():
    _initialised = False

    def __init__(self, camera=None, log_level=logging.INFO):
        self._initialised = False
        self._warnings = []

        self._image_pipe = HopperPipe(HopperPipeType.IN, "robot", "camera")
        self._image_pipe.open()

        setup_logging(log_level)

        if type(self)._initialised:
            raise RuntimeError("Camera object can only be initialised once")

        self.camera = vision.RoboConPiCamera() if camera is None else camera()
        if not isinstance(self.camera, vision.Camera):
            raise ValueError("camera must inherit from vision.Camera")
        self.res = self.camera.res

        self._vision = vision.Vision(self.zone, camera=self.camera, image_pipe=self._image_pipe)

        type(self)._initialised = True

    def see(self) -> vision.Detections:
        """Take a photo, detect markers in scene, attach RoboCon specific
        properties"""
        return self._vision.detect_markers()

    def __del__(self):
        """Frees hardware resources held by the vision object"""
        try:
            self._image_pipe.close()
        except:
            pass

        # If vision never was initialised this creates confusing errors
        # so check that it is initialised first
        if hasattr(self, "_vision"):
            self._vision.stop()
        type(self)._initialised = False

