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
from smbus2 import SMBus

from robot import vision
from robot.cytron import CytronBoard
from robot.greengiant import GreenGiantInternal, GreenGiantGPIOPinList, GreenGiantMotors, _GG_SERVO_PWM_BASE, _GG_GPIO_PWM_BASE, _GG_GPIO_GPIO_BASE, _GG_SERVO_GPIO_BASE
from robot.game_config import TEAM
from . import game_config
from robot.game_config import POEM_ON_STARTUP

from hopper.client import *
from hopper.common import *

_logger = logging.getLogger("robot")

# path to file with status of USB program copy,
# if this exists it is because the code on the robot has been copied from the robotusb
# this boot cycle. This is to highlight weird behaviour in the arena
COPY_STAT_FILE = "/tmp/usb_file_uploaded"


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


class Robot():
    """Class for initialising and accessing robot hardware"""

    _initialised = False

    def __init__(self,
                 wait_for_start=True,
                 camera=None,
                 max_motor_voltage=6,
                 logging_level=logging.INFO,
                 start_enable_12v = True,
                 start_enable_5v = True,
                 ):

        self.zone = game_config.TEAM.RED
        self.mode = "competition"
        self._max_motor_voltage = max_motor_voltage

        self._initialised = False
        self._start_pressed = False
        self._warnings = []

        # Initialize a RcMuxClient and open the start pipe 
        self._hopper_client = HopperClient()
        self._start_pipe = PipeName((PipeType.OUTPUT, "start-button", "robot"), "/home/pi/pipes")
        self._hopper_client.open_pipe(self._start_pipe, delete=True, create=True, blocking=True)   # Make sure to use blocking mode, otherwise start button code fails

        self._log_pipe = PipeName((PipeType.INPUT, "log", "robot"), "/home/pi/pipes")

        # Close stdout and stderr
        os.close(1)
        os.close(2)

        # ...and open a pipe in its place
        self._hopper_client.open_pipe(self._log_pipe, delete=True, create=True)
        os.dup(self._hopper_client.get_pipe_by_pipe_name(self._log_pipe).fd)

        self._parse_cmdline()

        setup_logging(logging_level)

        # check if copy stat file exists and read it if it does then delete it
        try:
            with open(COPY_STAT_FILE, "r") as f:
                _logger.info("Robot code copied %s from USB\n", f.read().strip())
            os.remove(COPY_STAT_FILE)
        except IOError:
            pass

        self.subsystem_init(camera, start_enable_12v, start_enable_5v)
        self.report_hardware_status()
        type(self)._initialised = True

        # Allows for the robot object to be set up and mutated before being set
        # up. Dangerous as it means the start info might not get loaded
        # depending on user code.
        if wait_for_start is True:
            start_data = self.wait_start()
            self.zone = start_data['zone']
            self.mode = start_data['mode']
        else:
            _logger.warning("Robot initalized but usercode running before"
                           "`robot.wait_start`. Robot will not wait for the "
                           "start button until `robot.wait_start` is called.")

    def subsystem_init(self, camera, start_enable_12v, start_enable_5v):
        """Allows for initalisation of subsystems after instansating `Robot()`
        Can only be called once"""
        if type(self)._initialised:
            raise RuntimeError("Robot object is acquires hardware locks for its"
                               " sole use and so can only be used once.")

        self.bus = SMBus(1)
        self._green_giant = GreenGiantInternal(self.bus)
        self._gg_version = self._green_giant.get_version()
        if self._gg_version >= 10:
            # enable power rails
            self._green_giant.set_motor_power(True)
            self.enable_12v = start_enable_12v
            self.enable_5v = start_enable_5v
            self._adc_max = 5
            # configure User IO Ports
            self.servos = GreenGiantGPIOPinList(self.bus, self._gg_version, self._adc_max, _GG_SERVO_GPIO_BASE, _GG_SERVO_PWM_BASE)
            self.gpio   = GreenGiantGPIOPinList(self.bus, self._gg_version, self._adc_max, _GG_GPIO_GPIO_BASE, _GG_GPIO_PWM_BASE)
            # configure motor drivers
            self.motors = GreenGiantMotors(self.bus, self._max_motor_voltage)
            ## thinks, perhaps this should be inherrent to using the motors and
            ## open load detection can be in there?
            self.motors.enable_motors(True)
        else:
            # power rails
            self._green_giant.set_motor_power(True)
            self._adc_max = self._green_giant.get_fvr_reading()
            # user IO
            self.servos = GreenGiantGPIOPinList(self.bus, self._gg_version, None,          None,           _GG_SERVO_PWM_BASE)
            self.gpio   = GreenGiantGPIOPinList(self.bus, self._gg_version, self._adc_max, _GG_GPIO_GPIO_BASE    , None)
            # configure motor drivers
            self.motors = CytronBoard(self._max_motor_voltage)

        self.camera = vision.RoboConPiCamera() if camera is None else camera()
        if not isinstance(self.camera, vision.Camera):
            raise ValueError("camera must inherit from vision.Camera")
        self.res = self.camera.res

        self._vision = vision.Vision(self.zone, camera=self.camera)

    def report_hardware_status(self):
        """Print out a nice log message at the start of each robot init with
        the hardware status"""

        battery_voltage = self._green_giant.get_battery_voltage()
        battery_str = "Battery Voltage: %.2fv" % battery_voltage
        # GG cannot read voltages above 12.2v
        if battery_voltage > 12.2:
            battery_str = "Battery Voltage: > 12.2v"
        if battery_voltage < 11.5:
            self._warnings.append("Battery voltage below 11.5v, consider "
                                  "changing for a charged battery")

        if self._gg_version < 3:
            self._warnings.append(
                "Green Giant version not 3 but instead {}".format(self._gg_version))

        camera_type_str = "Camera:    {}".format(
            self.camera.__class__.__name__)

        # Adds a secret every now and again!
        POEM_ON_STARTUP.on_startup(_logger,random)

        # print report of hardware
        _logger.info("------HARDWARE REPORT------")
        #_logger.info("Time:   %s", datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        # no RTC on new boards, perhaps use a "run number" increment instead?
        _logger.info("Patch Version:     ")
        _logger.info(battery_str)
        #_logger.info("ADC Max:           %.2fv", self._adc_max)
        _logger.info("Robocon Board:   Yes (v%d)", self._gg_version)
        if self._gg_version <= 3:
            _logger.info("Motor Driver:  Cytron Board")
        else:
            _logger.info("Motor Driver:  PiLow Cover")
            # check and report open load here, warning race condition, is battery power on long enough?
            # Thinks that we just motor enable/disable and leave motor power always on?
        _logger.info(camera_type_str)
        _logger.info("---------------------------")

        for warning in self._warnings:
            _logger.warning("WARNING: %s", warning)

        if not self._warnings:
            _logger.info("Hardware looks good")

    @property
    def enable_motors(self):
        """Return if motors are currently enabled

        For the GG board this will be the state of the 12v line, which we cannot query,
        so return what it was set to.

        For the PiLow series the Motors have both a power control and a enable. Generally
        the Power should not be switched on and off, just the enable bits. The power may
        be tripped in extreme circumstances. I guess that here we want to report any
        reason for  the motors not working, which includes power and enable

        """
        if self._gg_version < 10:
            return self._green_giant.enable_12v
        else:
            return self._green_giant.get_motorpwr() and self._green_giant.get_enable()

    @enable_motors.setter
    def enable_motors(self, on):
        """An nice alias for set_12v"""
        if self._version < 10:
            return self._green_giant.enable_motors(on)

    @property
    def enable_12v(self):
        return self._green_giant.get_12v_acc_power()

    @enable_12v.setter
    def enable_12v(self, on):
        self._green_giant.set_12v_acc_power(on)

    @property
    def enable_5v(self):
        return self._green_giant.get_5v_acc_power()

    @enable_5v.setter
    def enable_5v(self, on):
        self._green_giant.set_5v_acc_power(on)

    def stop(self):
        """Stops the robot and cuts power to the motors.

        does not touch the servos position.
        """
        self.enable_12v = False
        self.motors.stop()

    def _parse_cmdline(self):
        """Parse the command line arguments"""
        parser = optparse.OptionParser()

        parser.add_option("--usbkey", type="string", dest="usbkey",
                          help="The path of the (non-volatile) user USB key")

        (options, _) = parser.parse_args()

        self.usbkey = options.usbkey

    def _wait_start_blink(self):
        """Blink status LED until start is pressed"""
        v = False
        while not self._start_pressed:
            time.sleep(0.2)
            self._green_giant.set_user_led(v)
            v = not v
        if self._gg_version < 10:
            # on GG keep main LED on to show the device is running
            self._green_giant.set_user_led(True)
        else:
            # for PiLow the board has its own Power LED, so this can be used by users
            self._green_giant.set_user_led(False)

    def _get_start_info(self):
        """Get the start infomation from the named pipe"""

        # This call blocks until the start info is read
        d = self._hopper_client.read(self._start_pipe).decode("utf-8")

        settings = json.loads(d)

        assert "zone" in settings, "zone must be in startup info"
        if settings["zone"] not in range(4):
            raise ValueError(
                "zone must be in range 0-3 inclusive -- value of %i is invalid"
                % settings["zone"])
        settings["zone"] = TEAM[f"T{settings['zone']}"]

        self._start_pressed = True

        return settings

    def wait_start(self):
        """Wait for the start signal to happen"""

        blink_thread = threading.Thread(target=self._wait_start_blink)
        blink_thread.start()

        _logger.info("\nWaiting for start signal...")

        # This blocks till we get start info
        start_info = self._get_start_info()

        _logger.info("Robot started!\n")

        return start_info

    def set_user_led(self, val=True):
       self._green_giant.set_user_led(val)

    def see(self, look_for=None) -> vision.Detections:
        """Take a photo, detect markers in sene, attach RoboCon specific
        properties"""
        return self._vision.detect_markers(look_for=look_for)

    def __del__(self):
        """Frees hardware resources held by the vision object"""
        logging.warning("Destroying robot object")
        # If vision never was initialled this creates confusing errors
        # so check that it is initialled first
        if hasattr(self, "_vision"):
            self._vision.stop()
        type(self)._initialised = False
