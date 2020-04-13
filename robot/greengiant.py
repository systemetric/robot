"""
A set of constants and interfaces for controlling the green giant over I2C
"""

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

OUTPUT = 0b00
INPUT = 0b01
INPUT_ANALOG = 0b10
INPUT_PULLUP = 0b11

_GG_I2C_ADDR = 0x8

# PWM
#        H  L
# PWM 1: 1, 2
# PWM 2: 3, 4
# PWM 3: 5, 6
# PWM 4: 7, 8
_GG_PWM_START = 1

_GG_PWM_CENTER = 374
_GG_PWM_PERCENT_HALF_RANGE = 125
_GG_PWM_HALF_RANGE = 224
_GG_PWM_MIN = _GG_PWM_CENTER - _GG_PWM_HALF_RANGE
_GG_PWM_MAX = _GG_PWM_CENTER + _GG_PWM_HALF_RANGE

# Analog Read
#        H   L
# Pin 1:  9, 10
# Pin 2: 11, 12
# Pin 3: 13, 14
# Pin 4: 15, 16
_GG_ANALOG_START = 9

# Control (Mode Set)
# Pin 1: 17
# Pin 2: 18
# Pin 3: 19
# Pin 4: 20
_GG_CONTROL_START = 17

# Digital Read/Write
# Pin 1: 21
# Pin 2: 22
# Pin 3: 23
# Pin 4: 24
_GG_DIGITAL_START = 21

# Status
# 25
_GG_STATUS = 25

# Unsafe
# 26

# Enable 12V
# 27
_GG_ENABLE_12V = 27

# Battery_V
# H   L
# 28, 29
_GG_BATTERY_V_H = 28
_GG_BATTERY_V_L = 29

# Fixed Voltage Reference @ 4.096
_GG_FVR_H = 30
_GG_FVR_L = 31

# Version number (== 2)
_GG_VERSION = 32


def read_high_low_data(bus, high, low):
    high_value = bus.read_byte_data(_GG_I2C_ADDR, high)
    low_value = bus.read_byte_data(_GG_I2C_ADDR, low)

    return low_value + (high_value << 8)


class GreenGiantInternal(object):
    def __init__(self, bus):
        self._bus = bus

    def set_12v(self, on):
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_12V, int(on))

    def set_status_led(self, on):
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_STATUS, int(on))

    def get_version(self):
        return self._bus.read_byte_data(_GG_I2C_ADDR, _GG_VERSION)

    def get_battery_voltage(self):
        return 804519.936 / read_high_low_data(self._bus, _GG_BATTERY_V_H, _GG_BATTERY_V_L)

    def get_fvr_reading(self):
        """# TODO work out what does fvr stand for? and doc string"""
        return 268173.312 / read_high_low_data(self._bus, _GG_FVR_H, _GG_FVR_L)


class GreenGiantGPIOPin(object):
    def __init__(self, bus, index, adc_max):
        self._bus = bus
        self._index = index
        self._mode = None
        self._adc_max = adc_max

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        self._mode = mode
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_CONTROL_START + self._index, mode)

    @property
    def digital(self):
        if self._mode != INPUT and self._mode != INPUT_PULLUP:
            raise ValueError("digital read attempted on non INPUT/INPUT_PULLUP pin")
        return bool(self._bus.read_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._index))

    @digital.setter
    def digital(self, value):
        if self._mode != OUTPUT:
            raise ValueError("digital write attempted on none OUTPUT pin")
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._index, int(value))

    @property
    def analog(self):
        if self._mode != INPUT_ANALOG and self._mode != INPUT_PULLUP:
            raise ValueError("analog read attempted on non INPUT_ANALOG/INPUT_PULLUP pin")

        command = _GG_ANALOG_START + (self._index * 2)

        return read_high_low_data(self._bus, command, command + 1) / float(0xFFC0) * self._adc_max


class GreenGiantPWM(object):
    def __init__(self, bus):
        self._bus = bus

    def __getitem__(self, index):
        if not (1 <= index <= 4):
            raise IndexError("pwm index must be between 1 and 4")
        index -= 1

        command = _GG_PWM_START + (index * 2)

        high = self._bus.read_byte_data(_GG_I2C_ADDR, command)
        low = self._bus.read_byte_data(_GG_I2C_ADDR, command + 1)
        value = low + (high << 8)

        return (value - _GG_PWM_CENTER) * 100 / _GG_PWM_PERCENT_HALF_RANGE

    def __setitem__(self, index, percent):
        if not (1 <= index <= 4):
            raise IndexError("pwm index must be between 1 and 4")
        index -= 1

        command = _GG_PWM_START + (index * 2)

        value = _GG_PWM_CENTER + (percent / 100 * _GG_PWM_PERCENT_HALF_RANGE)
        value = clamp(value, _GG_PWM_MIN, _GG_PWM_MAX)

        low = value & 0xFF
        high = value >> 8

        self._bus.write_byte_data(_GG_I2C_ADDR, command, high)
        self._bus.write_byte_data(_GG_I2C_ADDR, command + 1, low)

    def off(self):
        for i in range(4):
            self.__setitem__(i + 1, 0)
