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

from . import vision
from datetime import datetime
from smbus2 import SMBus
from robot.cytron import CytronBoard
from robot.greengiant import GreenGiantInternal, GreenGiantGPIOPin, GreenGiantPWM

logger = logging.getLogger("robot")

# path to file with status of USB program copy,
# if this exists it is output in logs and then deleted
COPY_STAT_FILE = "/root/COPYSTAT"

def setup_logging():
    """Apply default settings for logging"""
    # (We do this by default so that our users
    # don't have to worry about logging normally)

    logger.setLevel(logging.INFO)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.INFO)

    fmt = logging.Formatter("%(message)s")
    handler.setFormatter(fmt)

    logger.addHandler(handler)


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

        self.subsystem_init()

        self.report_harware_status()

        # Allows for the robot object to be set up and muated before being
        # started
        if wait_for_start:
            self.wait_start()
        else:
            logger.warn(
                "Robot initalized but user code running before wait_start")

    def report_harware_status(self):
        """Print out a nice log message at the start of each robot init with
        the hardware status"""

        battery_voltage = self._green_giant.get_battery_voltage()
        battery_str = "Battery Voltage:   %.2fv" % battery_voltage
        # we cannot read voltages above 12.2v
        if battery_voltage > 12.2:
            battery_str = "Battery Voltage:   > 12.2v"
        if battery_voltage < 11.5:
            self._warnings.append(
                "Battery voltage below 11.5v, consider changing for a charged battery")

        if self._gg_version != 2:
            self._warnings.append(
                "Green Giant version not 2 but instead {}".format(self._gg_version))

        if self.vision._using_usb_cam:
            vision_str = "Camera:            USB"
        else:
            vision_str = "Camera:       PiCamera"

        # print report of hardware
        logger.info("------HARDWARE REPORT------")
        logger.info("Time:   %s",
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        logger.info("Patch Version:     0")
        logger.info(battery_str)
        logger.info("ADC Max:           %.2fv", self._adc_max)
        logger.info("Green Giant Board: Yes (v%d)", self._gg_version)
        logger.info("Cytron Board:      Yes")
        logger.info(vision_str)
        logger.info("---------------------------")

        for warning in self._warnings:
            logger.warn("WARNING: %s" % warning)

        if not self._warnings:
            logger.info("Hardware looks good")

    @pre_init
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

        self._adc_max = self._green_giant.get_fvr_reading()
        self._gg_version = self._green_giant.get_version()

        self.gpio = [None]
        for i in range(4):
            self.gpio.append(GreenGiantGPIOPin(self.bus, i, self._adc_max))

        self.motors = CytronBoard()

        self.vision = vision.Vision(self.zone)
        self.camera = self.vision.camera

        self._initialised = True

    def stop(self):
        """Stops the robot and cuts power to the motors"""
        self._green_giant.set_12v(False)
        self.motors.stop()

    def off(self):
        """Turns motors off"""
        # TODO is this documented as a public method? should we also have on?
        # TODO the difference between this and Robot.stop is confusing
        for motor in self.motors:
            motor.off()

    def _parse_cmdline(self):
        """Parse the command line arguments"""
        parser = optparse.OptionParser()

        parser.add_option("--usbkey", type="string", dest="usbkey",
                          help="The path of the (non-volatile) user USB key")

        parser.add_option("--startfifo", type="string", dest="startfifo",
                          help="""The path of the fifo which start information
                                  will be received through""")
        (options, _) = parser.parse_args()

        self.usbkey = options.usbkey
        self.startfifo = options.startfifo

    def wait_start_blink(self):
        """When the robot object has been initalized asynchronously flash the
        status led"""
        v = False
        while not self._start_pressed:
            time.sleep(0.2)
            self._green_giant.set_status_led(v)
            v = not v
        self._green_giant.set_status_led(True)

    def wait_start(self):
        """Wait for the start signal to happen"""

        if self.startfifo is None:
            self._start_pressed = True

            logger.info("\nNo startfifo so using defaults (Zone: {})\n".format(
                        self.zone))
            return

        t = threading.Thread(target=self.wait_start_blink)
        t.start()

        logger.info("\nWaiting for start signal...")

        f = open(self.startfifo, "r")
        d = f.read()
        f.close()

        self._start_pressed = True

        j = json.loads(d)

        for prop in ("zone"):
            if prop not in j:
                raise ValueError("'{}' must be in startup info".format(prop))
            setattr(self, prop, j[prop])

        if self.zone < 0 or self.zone > 3:
            raise ValueError(
                "zone must be in range 0-3 inclusive -- value of %i is invalid"
                % self.zone)

        logger.info("Robot started!\n")

    # TODO Document on website
    # TODO maybe this is the nice way to deal with signal init in the
    # preprocessor init?
    # TODO there is alot of repition here
    @property
    def bounding_box(self):
        """Weather we draw boxes around detected markers"""
        return self.vision.post_processor.settings["bounding_box"].is_set()

    @bounding_box.setter
    def bounding_box(self, value):
        """Weather we draw boxes around detected markers"""
        signal = self.vision.post_processor.settings["bounding_box"]
        self.vision.assign_signal(signal, value)

    @property
    def usb_stick(self):
        """Weather we save images to a usb stick"""
        return self.vision.post_processor.settings["usb_stick"].is_set()

    @usb_stick.setter
    def usb_stick(self, value):
        """Weather we save images to a usb stick"""
        signal = self.vision.post_processor.settings["usb_stick"]
        self.vision.assign_signal(signal, value)

    @property
    def send_to_sheep(self):
        """Weather we send images to sheep"""
        return self.vision.post_processor.settings["send_to_sheep"].is_set()

    @send_to_sheep.setter
    def send_to_sheep(self, value):
        """Weather we send images to sheep"""
        signal = self.vision.post_processor.settings["send_to_sheep"]
        self.vision.assign_signal(signal, value)

    @property
    def save(self):
        """Weather we save images to `\\tmp\\col_image.jpg`"""
        return self.vision.post_processor.settings["save"].is_set()

    @save.setter
    def save(self, value):
        """Weather we save images to `\\tmp\\col_image.jpg`"""
        signal = self.vision.post_processor.settings["save"]
        self.vision.assign_signal(signal, value)

    def see(self):
        if not hasattr(self, "vision"):
            raise NoCameraPresent()

        return self.vision.detect_markers()
