"""
The module containing the `Robot` class

Mainly provides init routine for the brain and binds attributes of the `Robot`
class to their respecitve classes
"""
import json
import sys
import optparse
import os
import glob
import logging
import time
import threading
from datetime import datetime
import pyudev

from smbus2 import SMBus

from robot.cytron import CytronBoard
from robot.greengiant import GreenGiantInternal, GreenGiantGPIOPin, GreenGiantPWM

from . import vision

logger = logging.getLogger("robot")

# path to file with status of USB program copy,
# if this exists it is output in logs and then deleted
COPY_STAT_FILE = "/root/COPYSTAT"


def setup_logging():
    """Apply default settings for logging"""
    # (We do this by default so that our users
    # don't have to worry about logging normally)

    logger.setLevel(logging.INFO)

    h = logging.StreamHandler(sys.stdout)
    h.setLevel(logging.INFO)

    fmt = logging.Formatter("%(message)s")
    h.setFormatter(fmt)

    logger.addHandler(h)


class NoCameraPresent(Exception):
    """Camera not connected."""

    def __str__(self):
        return "No camera found."


class AlreadyInitialised(Exception):
    """The robot has been initialised twice"""

    def __str__(self):
        return "Robot object can only be initialised once."


class UnavailableAfterInit(Exception):
    """The called function is unavailable after init()"""

    def __str__(self):
        return "The called function is unavailable after init()"


def pre_init(f):
    """Decorator for functions that may only be called before init()"""

    def g(self, *args, **kw):
        if self._initialised:
            raise UnavailableAfterInit()

        return f(self, *args, **kw)

    return g


class Robot(object):
    """Class for initialising and accessing robot hardware"""

    def __init__(self,
                 quiet=False,
                 wait_for_start=True,
                 config_logging=True,
                 use_usb_camera=False):

        self._use_usb_camera = use_usb_camera
        self._quiet = quiet

        if config_logging:
            setup_logging()

        self.zone = 0
        self.mode = "dev"
        self.arena = "A"

        self._initialised = False
        self._start_pressed = False
        self._warnings = []

        self._parse_cmdline()

        # check if copy stat file exists and read it if it does then delete it
        # What is this for?
        try:
            with open(COPY_STAT_FILE, "r") as f:
                logger.info("Copied %s from USB\n" % f.read().strip())
            os.remove(COPY_STAT_FILE)
        except IOError:
            pass

        # register components
        self.subsystem_init()

        self.report_harware_status()

        # Allows for the robot object to be set up and muated before being
        # started
        if wait_for_start:
            self.wait_start()
        else:
            logger.warn("Robot initalized but user code running before wait_start")

    def report_harware_status(self):
        """Print out a nice log message at the start of each robot init with
        the hardware status"""

        battery_voltage = self._green_giant.get_battery_voltage()
        battery_str = "Battery Voltage:   %.2fv" % battery_voltage
        # we cannot read voltages above 12.2v
        if battery_voltage > 12.2:
            battery_str = "Battery Voltage:   > 12.2v"
        if battery_voltage < 11.5:
            self._warnings.append("Battery voltage below 11.5v, consider changing for a charged battery")

        self._adc_max = self._green_giant.get_fvr_reading()

        self._gg_version = self._green_giant.get_version()
        if self._gg_version != 2:
            self._warnings.append("Green Giant version not 2 but instead {}".format(self._gg_version))

        if self.vision._using_usb_cam:
            vision_str = "Camera:            USB"
        else:
            vision_str = "Camera:       PiCamera"

        # print report of hardware
        logger.info("------HARDWARE REPORT------")
        logger.info("Time:   %s" % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        logger.info("Patch Version:     0")
        logger.info(battery_str)
        logger.info("ADC Max:           %.2fv" % self._adc_max)
        logger.info("Green Giant Board: Yes (v%d)" % self._gg_version)
        logger.info("Cytron Board:      Yes")
        logger.info(vision_str)
        logger.info("---------------------------")

        for warning in self._warnings:
            logger.warn("WARNING: %s" % warning)

        if not self._warnings:
            logger.info("Hardware looks good")

    def stop(self):
        """
        Stops the robot and cuts power to the motors
        """
        self._green_giant.set_12v(False)
        self.motors.stop()

    def subsystem_init(self):
        """
        Allows for the user to initalize the subsystems after the robot object
        """
        if self._initialised:
            raise AlreadyInitialised()

        self.bus = SMBus(1)
        self._green_giant = GreenGiantInternal(self.bus)
        self._green_giant.set_12v(True)
        self.servos = GreenGiantPWM(self.bus)

        self.gpio = [None]
        for i in range(4):
            self.gpio.append(GreenGiantGPIOPin(self.bus, i, self._adc_max))

        self.motors = CytronBoard()

        self.vision = vision.Vision(self.mode, self.arena, self.zone)

        self._initialised = True

    def off(self):
        """Turns motors off"""
        #TODO is this obsolite code
        for motor in self.motors:
            motor.off()

    def _parse_cmdline(self):
        """Parse the command line arguments"""
        parser = optparse.OptionParser()

        parser.add_option("--usbkey", type="string", dest="usbkey",
                          help="The path of the (non-volatile) user USB key")

        parser.add_option("--startfifo", type="string", dest="startfifo",
                          help="The path of the fifo which start information will be received through")
        (options, _) = parser.parse_args()

        self.usbkey = options.usbkey
        self.startfifo = options.startfifo

    def wait_start_blink(self):
        v = False
        while not self._start_pressed:
            time.sleep(0.2)
            self._green_giant.set_status_led(v)
            v = not v
        self._green_giant.set_status_led(True)

    # noinspection PyUnresolvedReferences
    def wait_start(self):
        """Wait for the start signal to happen"""

        if self.startfifo is None:
            self._start_pressed = True

            logger.info("\nNo startfifo so using defaults (Zone: 0, Mode: dev, Arena: A)\n")
            return

        t = threading.Thread(target=self.wait_start_blink)
        t.start()

        logger.info("\nWaiting for start signal...")

        f = open(self.startfifo, "r")
        d = f.read()
        f.close()

        self._start_pressed = True

        j = json.loads(d)

        for prop in ["zone", "mode", "arena"]:
            if prop not in j:
                raise ValueError("'{}' must be in startup info".format(prop))
            setattr(self, prop, j[prop])

        if self.mode not in ["comp", "dev"]:
            raise ValueError("mode of '%s' is not supported -- must be 'comp' or 'dev'" % self.mode)
        if self.zone < 0 or self.zone > 3:
            raise ValueError("zone must be in range 0-3 inclusive -- value of %i is invalid" % self.zone)
        if self.arena not in ["A", "B"]:
            raise ValueError("arena must be A or B")

        logger.info("Robot started!\n")

    # noinspection PyUnresolvedReferences
    def see(self, res=(640, 480), save=True):
        if not hasattr(self, "vision"):
            raise NoCameraPresent()

        return self.vision.see(res, save)
