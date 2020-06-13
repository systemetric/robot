"""A set of constants and interfaces for controlling the green giant over I2C"""

def clamp(n, smallest, largest):
    """Returns N if in bounds else returns the exceeded bound"""
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


def read_high_low_data(bus, high_addr, low_addr):
    """Fetches and combines data stored across two bytes"""
    high_value = bus.read_byte_data(_GG_I2C_ADDR, high_addr)
    low_value = bus.read_byte_data(_GG_I2C_ADDR, low_addr)

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
        """Get the fixed voltage reading"""
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
        if self._mode not in (INPUT, INPUT_PULLUP):
            raise IOError(f"Digital read attempted on pin {self._index} "
                          f"requiring pin_mode in INPUT, INPUT_PULLUP but "
                          f"instead pin_mode is {self._mode}")
        return bool(self._bus.read_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._index))

    @digital.setter
    def digital(self, value):
        if self._mode != OUTPUT:
            raise IOError(f"Digital write attempted on pin {self._index} "
                          f"requiring pin_mode OUTPUT but instead pin_mode is "
                          f"{self._mode}")

        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._index, int(value))

    @property
    def analog(self):
        """Reads an analog value from the ADC and converts it to a voltage"""
        if self._mode not in (INPUT_ANALOG, INPUT_PULLUP):
            raise IOError(f"Analog read attempted on pin {self._index} "
                          f"requiring pin_mode in INPUT_ANALOG, INPUT_PULLUP "
                          f"but instead pin_mode is {self._mode}")

        # We have a 10 bit ADC this is the maximum value we could read from it
        raw_adc_max = 0xFFC0

        analog_addr = _GG_ANALOG_START + (self._index * 2)
        raw_adc_value = read_high_low_data(self._bus, analog_addr, analog_addr + 1)

        return  (raw_adc_value / raw_adc_max) * self._adc_max


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
