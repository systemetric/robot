def clamp(n, smallest, largest): return max(smallest, min(n, largest))


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

# Signed range of PWM value sent to GG board
# -_GG_PWM_SIGN_RANGE <= x <= _GG_PWM_SIGN_RANGE
# _GG_PWM_SIGN_RANGE = 256
# Offset to PWM value to make everything positive
# _GG_PWM_OFFSET = _GG_PWM_SIGN_RANGE
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

# Unsafe
# 26

# Enable 12V
# 27
_GG_ENABLE_12V = 27

# Battery_V
# H   L
# 28, 29


class GreenGiantInternal(object):
    def __init__(self, bus):
        self._bus = bus

    def set_12v(self, on):
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_12V, int(on))


class GreenGiantGPIOPin(object):
    def __init__(self, bus, index):
        self._bus = bus
        self._index = index
        self._mode = None

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

        high = self._bus.read_byte_data(_GG_I2C_ADDR, command)
        low = self._bus.read_byte_data(_GG_I2C_ADDR, command + 1)

        return low + (high << 7)


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
        value = low + (high << 7)

        return (value - _GG_PWM_CENTER) * 100 / _GG_PWM_PERCENT_HALF_RANGE

    def __setitem__(self, index, percent):
        if not (1 <= index <= 4):
            raise IndexError("pwm index must be between 1 and 4")
        index -= 1

        command = _GG_PWM_START + (index * 2)

        value = _GG_PWM_CENTER + (percent / 100 * _GG_PWM_PERCENT_HALF_RANGE)
        value = clamp(value, _GG_PWM_MIN, _GG_PWM_MAX)

        print index, value

        low = value & 0x7F
        high = value >> 7

        self._bus.write_byte_data(_GG_I2C_ADDR, command, high)
        self._bus.write_byte_data(_GG_I2C_ADDR, command + 1, low)

    def off(self):
        for i in range(4):
            self.__setitem__(i + 1, 0)
