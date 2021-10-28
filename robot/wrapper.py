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

from datetime import datetime
from smbus2 import SMBus

from robot import vision
from robot.cytron import CytronBoard
from robot.greengiant import GreenGiantInternal, GreenGiantGPIOPinList, GreenGiantPWM

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
                 logging_level=logging.INFO):

        self.zone = 0
        self.mode = "competition"
        self._max_motor_voltage = max_motor_voltage

        self._initialised = False
        self._start_pressed = False
        self._warnings = []

        self._parse_cmdline()

        setup_logging(logging_level)

        # check if copy stat file exists and read it if it does then delete it
        try:
            with open(COPY_STAT_FILE, "r") as f:
                _logger.info("Robot code copied %s from USB\n", f.read().strip())
            os.remove(COPY_STAT_FILE)
        except IOError:
            pass

        self.subsystem_init(camera)
        self.report_hardware_status()
        self.enable_12v = True
        type(self)._initialised = True

        # Allows for the robot object to be set up and mutated before being set
        # up. Dangerous as it means the start info might not get loaded
        # depending on user code.
        if wait_for_start is True:
            start_data = self.wait_start()
            self.zone = start_data.zone
            self.mode = start_data.mode
        else:
            _logger.warning("Robot initalized but usercode running before"
                           "`robot.wait_start`. Robot will not wait for the "
                           "start button until `robot.wait_start` is called.")

    def subsystem_init(self, camera):
        """Allows for initalisation of subsystems after instansating `Robot()`
        Can only be called once"""
        if type(self)._initialised:
            raise RuntimeError("Robot object is acquires hardware locks for its"
                               " sole use and so can only be used once.")

        self.bus = SMBus(1)
        self._green_giant = GreenGiantInternal(self.bus)
        self._adc_max = self._green_giant.get_fvr_reading()
        self._gg_version = self._green_giant.get_version()

        self.servos = GreenGiantPWM(self.bus)
        self.gpio = GreenGiantGPIOPinList(self.bus, self._adc_max)
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
        battery_str = "Battery Voltage:   %.2fv" % battery_voltage
        # we cannot read voltages above 12.2v
        if battery_voltage > 12.2:
            battery_str = "Battery Voltage:   > 12.2v"
        if battery_voltage < 11.5:
            self._warnings.append("Battery voltage below 11.5v, consider "
                                  "changing for a charged battery")

        if self._gg_version != 3:
            self._warnings.append("Green Giant version not 3 but instead {}".format(self._gg_version))

        camera_type_str = "Camera:            {}".format(self.camera.__class__.__name__)


        #Adds the secret poem every now and then!
        if random.randint(0,100) == 1:
            _logger.info("Today your task is a challenging one")
            _logger.info("Gifts for the wizard and deliveries to run")
            _logger.info("But due to the unfortunate timing you can not go")
            _logger.info("So you have sent a robot with gifts in tow")
            _logger.info("You start in your country with your gifts around")
            _logger.info("Starting in your home (where ever it is found)")
            _logger.info("Then taking gifts from your robots zone ")
            _logger.info("Delivering it to the wizard on its own")
            _logger.info("To the road is good and to the emerald palace is ideal ")
            _logger.info("And if in another country you get some but a point they will steal")
            _logger.info("There are many things that are to be considered")
            _logger.info("But remember to bring your gifts for the wizard")

        # print report of hardware
        _logger.info("------HARDWARE REPORT------")
        _logger.info("Time:   %s", datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        _logger.info("Patch Version:     0")
        _logger.info(battery_str)
        _logger.info("ADC Max:           %.2fv", self._adc_max)
        _logger.info("Green Giant Board: Yes (v%d)", self._gg_version)
        _logger.info("Cytron Board:      Yes")
        _logger.info("Motor Voltage:     %d", self._max_motor_voltage)
        _logger.info(camera_type_str)
        _logger.info("---------------------------")

        for warning in self._warnings:
            _logger.warning("WARNING: %s", warning)

        if not self._warnings:
            _logger.info("Hardware looks good")

    @property
    def enable_12v(self):
        """Return if 12v is currently enabled

        I (Edwin Shepherd) can't query this from the GG for some reason? but can
        on Jacks Fallens? @will can you test this on a more default brain?

        The code bellow seems to make my pi reboot (most of the time)
        The code will make the OS, blank screen then go to the rainbow
        bootscreen almost instantly. Doesn't seem to matter if it is run as
        root.

        I have plugged a scope into the 5V rail to make sure that the pi
        wasn't suddenly losing power and it doesn't seem to be, maybe I'm
        missing the edge. I think its software on the pi?

        On Jacks BB the bits doesn't change when read back even though its set
        and unset

        import time

        from smbus2 import SMBus

        I2C_ADDR = 0x8
        ENABLE_12V_REGISTER = 27
        bus = SMBus(1)

        for state in (True, False, True):
            print("setting state to {}".format(state))
            bus.write_byte_data(I2C_ADDR, ENABLE_12V_REGISTER, int(state))
            time.sleep(1)
            print("{0:b}".format(bus.read_byte_data(I2C_ADDR, ENABLE_12V_REGISTER)))
        """
        return self._green_giant.enabled_12v

    @enable_12v.setter
    def enable_12v(self, on):
        """An nice alias for set_12v"""
        return self._green_giant.set_12v(on)

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

        parser.add_option("--startfifo", type="string", dest="startfifo",
                          help="""The path of the fifo which start information
                                  will be received through""")
        (options, _) = parser.parse_args()

        self.usbkey = options.usbkey
        self.startfifo = options.startfifo

    def _wait_start_blink(self):
        """Blink status LED until start is pressed"""
        v = False
        while not self._start_pressed:
            time.sleep(0.2)
            self._green_giant.set_status_led(v)
            v = not v
        self._green_giant.set_status_led(True)

    def _get_start_info(self):
        """Get the start infomation from the fifo which was passed as an arg"""
        f = open(self.startfifo, "r")
        d = f.read()
        f.close()

        self._start_pressed = True

        settings = json.loads(d)

        assert "zone" in settings, "zone must be in startup info"
        if self.zone not in range(3):
            raise ValueError(
                "zone must be in range 0-3 inclusive -- value of %i is invalid"
                % self.zone)

        self._start_pressed = True

        return settings

    def wait_start(self):
        """Wait for the start signal to happen"""

        if self.startfifo is None:
            self._start_pressed = True
            _logger.info("No startfifo so using defaults (Zone: {})".format(self.zone))
            return

        blink_thread = threading.Thread(target=self._wait_start_blink)
        blink_thread.start()

        _logger.info("\nWaiting for start signal...")

        # This blocks till we get start info
        start_info = self._get_start_info()

        _logger.info("Robot started!\n")

        return start_info

    def see(self) -> vision.Detections:
        """Take a photo, detect markers in sene, attach RoboCon specific
        properties"""
        return self._vision.detect_markers()

    def __del__(self):
        """Frees hardware resources held by the vision object"""
        logging.warning("Destroying robot object")
        # If vision never was initialled this creates confusing errors
        # so check that it is initialled first
        if hasattr(self, "_vision"):
            self._vision.stop()
        type(self)._initialised = False
