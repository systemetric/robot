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

from robot.cytron import CytronBoard, DEFAULT_MOTOR_CLAMP
from robot.greengiant import GreenGiantInternal, GreenGiantGPIOPin, GreenGiantPWM

from . import vision

logger = logging.getLogger("sr.robot")

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
                 init=True,
                 config_logging=True,
                 use_usb_camera=False,
                 motor_max=DEFAULT_MOTOR_CLAMP,
                 servo_defaults=None):

        if config_logging:
            setup_logging()

        self._use_usb_camera = use_usb_camera

        self._initialised = False
        self._quiet = quiet

        self._warnings = []

        self._parse_cmdline()

        self.motor_max = motor_max

        # check if copy stat file exists and read it if it does then delete it
        try:
            with open(COPY_STAT_FILE, "r") as f:
                logger.info("Copied %s from USB\n" % f.read().strip())
            os.remove(COPY_STAT_FILE)
        except IOError:
            pass

        # register components
        bus = SMBus(1)
        self._internal = GreenGiantInternal(bus)
        self._internal.set_12v(True)
        self._gg_version = self._internal.get_version()

        # print report of hardware
        logger.info("------HARDWARE REPORT------")
        logger.info("Time:   %s" % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        logger.info("Patch Version:     2 (USBCAM)")

        # display battery voltage and warnings associated with it
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

        # gpio init
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

            if servo_defaults is not None:
                for servo, position in servo_defaults.iteritems():
                    self.servos[servo] = position

            self._start_pressed = False
            self.wait_start()

    """
    Stops the robot and cuts power to the motors
    """
    def stop(self):
        self.motors.stop()
        self._internal.set_12v(False)

    @classmethod
    def setup(cls, quiet=False, config_logging=True, use_usb_camera=False):
        if config_logging:
            setup_logging()

        logger.debug("Robot.setup( quiet = %s )", str(quiet))
        return cls(init=False,
                   quiet=quiet,
                   # Logging is already configured
                    config_logging=False,
                   use_usb_camera=use_usb_camera)

    """
    Initialises motors, pi cam and pwm
    """
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

    """
    Turns motors off
    """
    def off(self):
        for motor in self.motors:
            motor.off()

    def _dump_devs(self):
        """Write a list of relevant devices out to the log"""
        # logger.info("Found the following devices:")

        self._dump_webcam()

        if self._gg_version != 3:
            self._warnings.append("Green Giant version not 3")
        logger.info("Green Giant Board: Yes (v%d)" % self._gg_version)
        logger.info("Cytron Board:      Yes")

    def _dump_webcam(self):
        """Write information about the webcam to stdout"""

        if not hasattr(self, "vision"):
            logger.info("Pi Camera:         No")
            self._warnings.append("No Pi Camera detected")
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
            self._start_pressed = True

            logger.info("\nNo startfifo so using defaults (Zone: 0, Mode: dev, Arena: A)\n")
            setattr(self, "zone", 0)
            setattr(self, "mode", "dev")
            setattr(self, "arena", "A")
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
                raise Exception("'{}' must be in startup info".format(prop))
            setattr(self, prop, j[prop])

        if self.mode not in ["comp", "dev"]:
            raise Exception("mode of '%s' is not supported -- must be 'comp' or 'dev'" % self.mode)
        if self.zone < 0 or self.zone > 3:
            raise Exception("zone must be in range 0-3 inclusive -- value of %i is invalid" % self.zone)
        if self.arena not in ["A", "B"]:
            raise Exception("arena must be A or B")

        logger.info("Robot started!\n")

    def _init_devs(self, bus):
        """Initialise the attributes for accessing devices"""

        # Motor boards
        self._init_motors()
        # Servo boards
        self._init_pwm(bus)

    def _init_motors(self):
        self.motors = CytronBoard(self.motor_max)

    def _init_pwm(self, bus):
        self.servos = GreenGiantPWM(bus)

    def _list_usb_devices(self, model, subsystem=None):
        """Create a sorted list of USB devices of the given type"""
        udev = pyudev.Context()
        devs = list(udev.list_devices(ID_MODEL=model, subsystem=subsystem))
        # Sort by serial number
        devs.sort(key=lambda x: x["ID_SERIAL_SHORT"])
        return devs

    def _init_usb_devices(self, model, ctor, subsystem=None):
        devs = self._list_usb_devices(model, subsystem)

        # Devices stored in a dictionary
        # Each device appears twice in this dictionary:
        #  1. Under its serial number
        #  2. Under an integer key.  Integers assigned by ordering
        #     boards by serial number.
        srdevs = {}

        n = 0
        for dev in devs:
            serialnum = dev["ID_SERIAL_SHORT"]

            if "BUSNUM" in dev:
                srdev = ctor(dev.device_node,
                             busnum=int(dev["BUSNUM"]),
                             devnum=int(dev["DEVNUM"]),
                             serialnum=serialnum)
            else:
                srdev = ctor(dev.device_node,
                             busnum=None,
                             devnum=None,
                             serialnum=serialnum)

            srdevs[n] = srdev
            srdevs[serialnum] = srdev
            n += 1

        return srdevs

    def _init_vision(self):
        if self._use_usb_camera:
            udev = pyudev.Context()
            cams = list(udev.list_devices(
                subsystem="video4linux",
                ID_USB_DRIVER="uvcvideo",
            ))

            if not cams:
                return

            camera = cams[0].device_node
        else:
            camera = None

        # Find libkoki.so:
        libpath = None
        if "LD_LIBRARY_PATH" in os.environ:
            for d in os.environ["LD_LIBRARY_PATH"].split(":"):
                l = glob.glob("%s/libkoki.so*" % os.path.abspath(d))

                if len(l):
                    libpath = os.path.abspath(d)
                    break
        if libpath is None:
            v = vision.Vision(camera, "/home/pi/libkoki/lib")  # /root/libkoki/lib
        else:
            v = vision.Vision(camera, libpath)

            srdevs[n] = srdev
            srdevs[serialnum] = srdev
            n += 1

    # noinspection PyUnresolvedReferences
    def see(self, res=(1296, 736), stats=False, save=True,
     bounding_box=True):
        if not hasattr(self, "vision"):
            raise NoCameraPresent()

        return self.vision.see(res=res,
                               mode=self.mode,
                               arena=self.arena,
                               stats=stats,
                               save=save,
                               zone=self.zone,
                               bounding_box_enable = bounding_box)
