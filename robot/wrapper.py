# Copyright Robert Spanton 2014
import json
import sys
import optparse
import os
import glob
import logging

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

        self._parse_cmdline()

        bus = SMBus(1)
        self._internal = GreenGiantInternal(bus)
        self._internal.set_12v(True)
        self._gg_version = self._internal.get_version()

        battery_voltage = self._internal.get_battery_voltage()
        battery_warning = ""
        if battery_voltage < 11.5:
            battery_warning = " [WARNING: battery voltage below 11.5V, consider changing for a charged battery]"
        logger.info("Battery voltage: %dV%s" % (battery_voltage, battery_warning))

        self.gpio = [None]
        for i in range(4):
            self.gpio.append(GreenGiantGPIOPin(bus, i))

        if init:
            self.init(bus)
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

        logger.info("Initialising hardware.")
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
        logger.info("Found the following devices:")

        self._dump_webcam()

        gg_warning = "" if self._gg_version == 2 else " [WARNING: version not 2]"
        print " - Green Giant Board (v%d)%s" % (self._gg_version, gg_warning)
        print " - Cytron Board"

    def _dump_webcam(self):
        """Write information about the webcam to stdout"""

        if not hasattr(self, "vision"):
            # No webcam
            return

        # For now, just display the fact we have a webcam
        logger.info(" - Webcam")

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

    # noinspection PyUnresolvedReferences
    def wait_start(self):
        """Wait for the start signal to happen"""
        logger.info("Waiting for start signal.")

        if self.startfifo is None:
            logger.info("No startfifo so using defaults (Zone: 0, Mode: dev, Arena: A)")
            setattr(self, "zone", 0)
            setattr(self, "mode", "dev")
            setattr(self, "arena", "A")
            return

        f = open(self.startfifo, "r")
        d = f.read()
        f.close()

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
