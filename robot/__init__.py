#!/usr/bin/python3
"""The robot module, provides an python interface to the RoboCon hardware and
April tags a marker recognition system. Also performs convince functions for use
by shepherd"""

import importlib

has_picamera = importlib.find_loader("picamera") is not None

if not has_picamera:
    import sys
    import fake_rpi

    sys.modules["RPi"] = fake_rpi.RPi
    sys.modules["RPi.GPIO"] = fake_rpi.RPi.GPIO
    sys.modules["picamera"] = fake_rpi.picamera
    sys.modules["smbus2"] = fake_rpi.smbus

import sys

# This log import configures our logging for us, but we don't want to
# provide it as part of this package.
import robot.log

from robot.wrapper import Robot, NoCameraPresent
from robot.greengiant import OUTPUT, INPUT, INPUT_ANALOG, INPUT_PULLUP, PWM_SERVO
from robot.vision import RoboConUSBCamera
from robot.game_config import (
    MARKER,
    BASE_MARKER,
    ARENA_MARKER,
    TARGET_MARKER,
    MARKER_TYPE,
    TEAM
)


MINIUM_VERSION = (3, 6)
if sys.version_info <= MINIUM_VERSION:
    raise ImportError(
        "Expected python {} but instead got {}".format(MINIUM_VERSION, sys.version_info)
    )

__all__ = (
    "Robot",
    "NoCameraPresent",
    "OUTPUT",
    "INPUT",
    "INPUT_ANALOG",
    "INPUT_PULLUP",
    "PWM_SERVO",
    "MARKER",
    "BASE_MARKER",
    "ARENA_MARKER",
    "POTATO_MARKER",
    "MARKER_TYPE",
    "TEAM",
    "RoboConUSBCamera"
)
