import sys
import logging
from smbus2 import SMBus

from robocon.brain.cytron import CytronBoard
from robocon.brain.greengiant import (
    GreenGiantInternal,
    GreenGiantGPIOPinList,
    GreenGiantMotors,
    _GG_SERVO_PWM_BASE,
    _GG_GPIO_PWM_BASE,
    _GG_GPIO_GPIO_BASE,
    _GG_SERVO_GPIO_BASE)

_logger = logging.getLogger("robot_io")

def setup_logging(level):
    """Display the just the message when logging events
    Sets the logging level to `level`"""
    _logger.setLevel(level)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    fmt = logging.Formatter("%(message)s")
    handler.setFormatter(fmt)

    _logger.addHandler(handler)

class IO():
    _initialised = False

    def __init__(self, max_motor_voltage=6, enable_12v=True, enable_5v=True, log_level=logging.INFO):
        self._max_motor_voltage = max_motor_voltage

        self._initialised = False
        self._warnings = []

        setup_logging(log_level)

        if type(self)._initialised:
            raise RuntimeError("IO object acquires hardware locks for its"
                               " sole use and so can only be used once.")

        self.bus = SMBus(1)
        self._green_giant = GreenGiantInternal(self.bus)
        self._gg_version = self._green_giant.get_version()
        if self._gg_version >= 10:
            # enable power rails
            self._green_giant.set_motor_power(True)
            self.enable_12v = enable_12v
            self.enable_5v = enable_5v
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

        type(self)._initialised = True

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

    def set_user_led(self, val=True):
       self._green_giant.set_user_led(val)

    def __del__(self):
        """Frees hardware resources held by the vision object"""
        logging.warning("Destroying robot object")
        type(self)._initialised = False

