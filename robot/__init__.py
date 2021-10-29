#!/usr/bin/python3
"""The robot module, provides an python interface to the RoboCon hardware and
April tags a marker recognition system. Also performs convince functions for use
by shepherd"""
import sys

# This log import configures our logging for us, but we don't want to
# provide it as part of this package.
import robot.log

from robot.wrapper import Robot, NoCameraPresent
from robot.greengiant import OUTPUT, INPUT, INPUT_ANALOG, INPUT_PULLUP
from robot.vision import MARKER_ARENA,MARKER_CUBE_WINKIE,MARKER_CUBE_GILLIKAN, MARKER_CUBE_QUADLING,MARKER_CUBE_MUNCHKIN, MARKER_DEFAULT, RoboConUSBCamera

MINIUM_VERSION = (3, 6)
if sys.version_info <= MINIUM_VERSION:
    raise ImportError("Expected python {} but instead got {}".format(
        MINIUM_VERSION, sys.version_info))

__all__ = ["Robot", "NoCameraPresent", "OUTPUT", "INPUT", "INPUT_ANALOG",
           "INPUT_PULLUP", "MARKER_ARENA","MARKER_CUBE_WINKIE","MARKER_CUBE_GILLIKIN", "MARKER_CUBE_QUADLING","MARKER_CUBE_MUNCHKIN", "MARKER_DEFAULT"]
