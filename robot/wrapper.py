# Copyright Robert Spanton 2014
import json
import sys
import optparse
import os
import glob
import logging
import time
import threading
from datetime import datetime

from smbus2 import SMBus

from robot.cytron import CytronBoard
from robot.greengiant import GreenGiantInternal, GreenGiantGPIOPin, GreenGiantPWM

from . import vision

logger = logging.getLogger("sr.robot")


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
                 init=True,
                 config_logging=True):
        if config_logging:
            setup_logging()

        self._initialised = False
        self._quiet = quiet

        self._warnings = []

        self._parse_cmdline()

        bus = SMBus(1)
        self._internal = GreenGiantInternal(bus)
        self._internal.set_12v(True)
        self._gg_version = self._internal.get_version()

        logger.info("------HARDWARE REPORT------")
        logger.info("Time:   %s" % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

        battery_voltage = self._internal.get_battery_voltage()
        battery_str = "Battery Voltage:   %.2fv" % battery_voltage
        # we cannot read voltages above 12.2v
        if battery_voltage > 12.2:
            battery_str = "Battery Voltage:   > 12.2v"
        if battery_voltage < 11.5:
            self._warnings.append("Battery voltage below 11.5v, consider changing for a charged battery")
        logger.info(battery_str)
        self._adc_max = self._internal.get_fvr_reading()
        logger.info("ADC Max:           %.2fv" % self._adc_max)

        self.gpio = [None]
        for i in range(4):
            self.gpio.append(GreenGiantGPIOPin(bus, i, self._adc_max))

        if init:
            self.init(bus)
            logger.info("---------------------------")
            if len(self._warnings) > 0:
                for warning in self._warnings:
                    logger.warn("WARNING: %s" % warning)
            else:
                logger.info("Hardware looks good")

            self._start_pressed = False
            self.wait_start()

    def stop(self):
        self.motors.stop()
        self._internal.set_12v(False)

    @classmethod
    def setup(cls, quiet=False, config_logging=True):
        if config_logging:
            setup_logging()

        logger.debug("Robot.setup( quiet = %s )", str(quiet))
        return cls(init=False,
                   quiet=quiet,
                   # Logging is already configured
                   config_logging=False)

    def init(self, bus):
        # Find and initialise hardware
        if self._initialised:
            raise AlreadyInitialised()

        logger.debug("Initialising hardware.")
        self._init_devs(bus)
        self._init_vision()

        if not self._quiet:
            self._dump_devs()

        self._initialised = True

    def off(self):
        for motor in self.motors:
            motor.off()

    def _dump_devs(self):
        """Write a list of relevant devices out to the log"""
        # logger.info("Found the following devices:")

        self._dump_webcam()

        if self._gg_version != 2:
            self._warnings.append("Green Giant version not 2")
        logger.info("Green Giant Board: Yes (v%d)" % self._gg_version)
        logger.info("Cytron Board:      Yes")

    def _dump_webcam(self):
        """Write information about the webcam to stdout"""

        if not hasattr(self, "vision"):
            logger.info("Pi Camera:         No")
            # No webcam
            return

        # For now, just display the fact we have a webcam
        logger.info("Pi Camera:         Yes")

    @staticmethod
    def _dump_usbdev_dict(devdict, name):
        """Write the contents of a device dict to stdout"""

        if len(devdict) == 0:
            return

        logger.info(" - %s:", name)

        for key, motor in devdict.iteritems():
            if not isinstance(key, int):
                continue

            logger.info("    %(index)s: %(motor)s",
                        {"index": key, "motor": motor})

    def _parse_cmdline(self):
        """Parse the command line arguments"""
        parser = optparse.OptionParser()

        parser.add_option("--usbkey", type="string", dest="usbkey",
                          help="The path of the (non-volatile) user USB key")

        parser.add_option("--startfifo", type="string", dest="startfifo",
                          help="The path of the fifo which start information will be received through")
        (options, args) = parser.parse_args()

        self.usbkey = options.usbkey
        self.startfifo = options.startfifo

    def wait_start_blink(self):
        v = False
        while not self._start_pressed:
            time.sleep(0.2)
            self._internal.set_status_led(v)
            v = not v
        self._internal.set_status_led(True)

    # noinspection PyUnresolvedReferences
    def wait_start(self):
        """Wait for the start signal to happen"""

        if self.startfifo is None:
            time.sleep(3)
            self._start_pressed = True

            logger.info("\nNo startfifo so using defaults (Zone: 0, Mode: dev, Arena: A)\n")
            setattr(self, "zone", 0)
            setattr(self, "mode", "dev")
            setattr(self, "arena", "A")
            return

        t = threading.Thread(target=self.wait_start_blink)
        t.start()

        logger.info("\nWaiting for start signal.\n")

        f = open(self.startfifo, "r")
        d = f.read()
        f.close()

        self._start_pressed = True

        j = json.loads(d)

        for prop in ["zone", "mode", "arena"]:
            if prop not in j:
                raise Exception("'{}' must be in startup info".format(prop))
            setattr(self, prop, j[prop])

        if self.mode not in ["comp", "dev"]:
            raise Exception("mode of '%s' is not supported -- must be 'comp' or 'dev'" % self.mode)
        if self.zone < 0 or self.zone > 3:
            raise Exception("zone must be in range 0-3 inclusive -- value of %i is invalid" % self.zone)
        if self.arena not in ["A", "B"]:
            raise Exception("arena must be A or B")

    def _init_devs(self, bus):
        """Initialise the attributes for accessing devices"""

        # Motor boards
        self._init_motors()
        # Servo boards
        self._init_pwm(bus)

    def _init_motors(self):
        self.motors = CytronBoard()

    def _init_pwm(self, bus):
        self.servos = GreenGiantPWM(bus)

    def _init_vision(self):
        # Find libkoki.so:
        libpath = None
        if "LD_LIBRARY_PATH" in os.environ:
            for d in os.environ["LD_LIBRARY_PATH"].split(":"):
                l = glob.glob("%s/libkoki.so*" % os.path.abspath(d))

                if len(l):
                    libpath = os.path.abspath(d)
                    break
        if libpath is None:
            v = vision.Vision("/home/pi/libkoki/lib")  # /root/libkoki/lib
        else:
            v = vision.Vision(libpath)

        self.vision = v

    # noinspection PyUnresolvedReferences
    def see(self, res=(640, 480), stats=False, save=True):
        if not hasattr(self, "vision"):
            raise NoCameraPresent()

        return self.vision.see(res=res,
                               mode=self.mode,
                               arena=self.arena,
                               stats=stats,
                               save=save,
                               zone=self.zone)
