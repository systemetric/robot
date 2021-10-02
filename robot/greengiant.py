"""A set of constants and interfaces for controlling the green giant over I2C"""

def clamp(value, smallest, largest):
    """Return `value` if in bounds else returns the exceeded bound"""
    return max(smallest, min(value, largest))

def _decrement_pin_index(index):
    """Validate then decrement the pin index.

    The pins internally are numbered from zero but externally from 1
    """
    valid_indexes = (1, 2, 3, 4)
    if index not in valid_indexes:
        raise ValueError(f"GPIO pin index must be in {valid_indexes}"
                         + f" but instead got {index}")
    index -= 1

    return index

OUTPUT = "OUTPUT"
INPUT = "INPUT"
INPUT_ANALOG = "INPUT_ANALOG"
INPUT_PULLUP = "INPUT_PULLUP"

_GG_GPIO_MASKS = {
    OUTPUT: 0b00,
    INPUT: 0b01,
    INPUT_ANALOG: 0b10,
    INPUT_PULLUP: 0b11,
}


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
    """Intertions for use with the user
    not intended to be part of the robot API"""
    def __init__(self, bus):
        self._bus = bus
        self.enabled_12v = False
        self.set_12v(self.enabled_12v)

    def set_12v(self, new_state):
        """Set the 12V and store the state on the Pi"""
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_12V, int(new_state))
        self.enabled_12v = new_state
        # Up to and including GreenGiant v3 there is no way of reading the sate of 
        # the 12v rail. This is more than a software change because to be useful
        # we would have to monitor the output of the high side switch

    def set_status_led(self, on):
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_STATUS, int(on))

    def get_version(self):
        return self._bus.read_byte_data(_GG_I2C_ADDR, _GG_VERSION)

    def get_battery_voltage(self):
        """TODO where does this magic number come from"""
        return 804519.936 / read_high_low_data(self._bus, _GG_BATTERY_V_H, _GG_BATTERY_V_L)

    def get_fvr_reading(self):
        """Return the fixed voltage reading
        TODO where does this magic number come from
        """
        return 268173.312 / read_high_low_data(self._bus, _GG_FVR_H, _GG_FVR_L)


class GreenGiantGPIOPin():
    def __init__(self, bus, index, adc_max):
        self._bus = bus
        self._index = index
        self._mode = None
        self._adc_max = adc_max
        self._digital_read_modes = (INPUT, INPUT_PULLUP, OUTPUT)
        self._analog_read_modes = (INPUT_ANALOG, INPUT_PULLUP)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        self._mode = mode
        mask = _GG_GPIO_MASKS[mode]
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_CONTROL_START + self._index, mask)

    @property
    def digital(self):
        if self._mode not in self._digital_read_modes:
            raise IOError(f"Digital read attempted on pin {self._index} "
                          f"requiring pin_mode in {self._digital_read_modes} but "
                          f"instead pin_mode is {self._mode}")
        return bool(self._bus.read_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._index))

    @digital.setter
    def digital(self, value):
        if self._mode is not OUTPUT:
            raise IOError(f"Digital write attempted on pin {self._index} "
                          f"requiring pin_mode {OUTPUT} but instead pin_mode is "
                          f"{self._mode}")

        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._index, int(value))

    @property
    def analog(self):
        """Reads an analog value from the ADC and converts it to a voltage"""
        if self._mode not in self._analog_read_modes:
            raise IOError(f"Analog read attempted on pin {self._index} "
                          f"requiring pin_mode in {self._analog_read_modes} "
                          f"but instead pin_mode is {self._mode}")

        # We have a 10 bit ADC this is the maximum value we could read from it
        raw_adc_max = 0xFFC0

        analog_addr = _GG_ANALOG_START + (self._index * 2)
        raw_adc_value = read_high_low_data(self._bus, analog_addr, analog_addr + 1)

        return  (raw_adc_value / raw_adc_max) * self._adc_max

    def __bool__(self):
        """Return a bool if in a digital mode or a float if in an analogue"""
        if self._mode in self._digital_read_modes:
            return self.digital

        raise ValueError(f"Tried to evaluate GPIO {self._index} as a True/False "
                         f"but current mode:{self._mode} is not in {self._digital_read_modes}")


class GreenGiantGPIOPinList():
    """A list of pins indexed from 1"""
    def __init__(self, bus, adc_max):
        self._list = [GreenGiantGPIOPin(bus, i, adc_max)
                      for i in range(4)]

    def __getitem__(self, index):
        return self._list[_decrement_pin_index(index)]

    def __setitem__(self, index, value):
        internal_index = _decrement_pin_index(index)
        self._list[internal_index] = value


class GreenGiantPWM():
    """An object implementing a descriptor protocol to control the servos"""
    def __init__(self, bus):
        self._bus = bus

    def __getitem__(self, index):
        index = _decrement_pin_index(index)

        command = _GG_PWM_START + (index * 2)

        high = self._bus.read_byte_data(_GG_I2C_ADDR, command)
        low = self._bus.read_byte_data(_GG_I2C_ADDR, command + 1)
        value = low + (high << 8)

        return (value - _GG_PWM_CENTER) * 100 / _GG_PWM_PERCENT_HALF_RANGE

    def __setitem__(self, index, percent):
        index = _decrement_pin_index(index)
        command = _GG_PWM_START + (index * 2)

        value = _GG_PWM_CENTER + (percent / 100 * _GG_PWM_PERCENT_HALF_RANGE)
        value = clamp(value, _GG_PWM_MIN, _GG_PWM_MAX)
        value = int(value)

        low = value & 0xFF
        high = value >> 8

        self._bus.write_byte_data(_GG_I2C_ADDR, command, high)
        self._bus.write_byte_data(_GG_I2C_ADDR, command + 1, low)

    def off(self):
        for i in range(4):
            self.__setitem__(i + 1, 0)
