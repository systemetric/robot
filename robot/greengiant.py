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
PWM_SERVO = "PWM_SERVO"

_GG_GPIO_MASKS = {
    OUTPUT: 0b000,
    INPUT: 0b001,
    INPUT_ANALOG: 0b010,
    INPUT_PULLUP: 0b011,
    PWM_SERVO: 0b100,
    # PWM_MARK_SPACE: 0b101,
    # ULTRASONIC: 0xb110
}





#__GG_MOTOR_ERROR_STATE_MASK = {
#    "GG_Motor_A_Open_Load":     1 << 0        # not always an error, may be weedy motors
#    "GG_Motor_A_Short":         1 << 1            # dead short or motor too power hungry for driver
#    "GG_Motor_A_Overheat":      1 << 2         # In excess of 165 Degrees at Junction - beware of the hot case
#    "GG_Motor_A_Short_To_Rail": 1 << 3   # connection detected between GND and motor or 12v and motor
#    "GG_Motor_B_Open_Load":     1 << 4
#    "GG_Motor_B_Short":         1 << 5
#    "GG_Motor_B_Overheat":      1 << 6
#    "GG_Motor_B_Short_To_Rail": 1 << 7
#}

#__GG_SYSTEM_ERROR_STATE_MASK = {
#
#    GG_System_5v_Fault: 1 << 0          # Short on GPIO or Servo overload
#    GG_System_MotorPower_Fault: 1 << 1  # Probably should not happen, but may with two high power motors
#    GG_System_12v_Fault: 1 << 2         # Short or overload on 12v Acc port
#    # avr overheat ?
#    # low power lockout ?
#}



_GG_I2C_ADDR = 0x08

_GG_SERVO_PWM_BASE = 1
_GG_GPIO_PWM_BASE = 48

# PWM (external ports numbered from zero on PiLow)
#        H  L
# PWM 1: 0, 1
# PWM 2: 2, 3
# PWM 3: 4, 5
# PWM 4: 6, 7

_GG_GG_PWM_CENTER = 374
_GG_GG_PWM_PERCENT_HALF_RANGE = 125
_GG_GG_PWM_HALF_RANGE = 224
_GG_GG_PWM_MIN = _GG_GG_PWM_CENTER - _GG_GG_PWM_HALF_RANGE
_GG_GG_PWM_MAX = _GG_GG_PWM_CENTER + _GG_GG_PWM_HALF_RANGE

# 1ms is 3000  # brain fade, check numbers, normal range = 1 to 2 ms (-100 ti 100) extended range is 0.8 to 2.2 (-140 to 140)?
_GG_PiLow_PWM_CENTER = 4500
_GG_PiLow_PWM_PERCENT_HALF_RANGE = 140
_GG_PiLow_PWM_HALF_RANGE = 2100
_GG_PiLow_PWM_MIN = _GG_PiLow_PWM_CENTER - _GG_PiLow_PWM_HALF_RANGE
_GG_PiLow_PWM_MAX = _GG_PiLow_PWM_CENTER + _GG_PiLow_PWM_HALF_RANGE


_GG_GPIO_GPIO_BASE = 9
_GG_SERVO_GPIO_BASE = 56
# Analog Read (external ports numbered from zero on PiLow)
#         H  L
# Pin 1:  0, 1
# Pin 2:  2, 3
# Pin 3:  4, 5
# Pin 4:  6, 7
_GG_ANALOG_START = 0

# Control (Mode Set - external ports numbered from zero on PiLow)
# Pin 1: 8
# Pin 2: 9
# Pin 3: 10
# Pin 4: 11
_GG_CONTROL_START = 8

# Digital Read/Write (external ports numbered from zero on PiLow)
# Pin 1: 12
# Pin 2: 13
# Pin 3: 14
# Pin 4: 15
_GG_DIGITAL_START = 12

# User LED
# 25
_GG_USER_LED = 25

# Watchdog, this will be the watchdog that needs pinging to tell the brain that Python is still running
# Currently this cannot be used due to the design of the code
# 26

# Enable motors , on the GG this also switches on the external 12v power
# 27
_GG_ENABLE_MOTORS = 27

# Battery_V
# H   L
# 28, 29
_GG_BATTERY_V_H = 28
_GG_BATTERY_V_L = 29

_GG_FVR_VOLTS = 4.096
_GG_BATTERY_MAX_READING = 3 * _GG_FVR_VOLTS
_GG_BATTERY_ADC_MAX = 65472


# Fixed Voltage Reference @ 4.096 - Cannot read this on the PiLow, maybe its pointless anyway
_GG_FVR_H = 30
_GG_FVR_L = 31

# Version number (3 is GreenGiant, >=10 is PiLow)
_GG_VERSION = 32


""" Additional features of the PiLow """
# Enables for the various power domains
_GG_ENABLE_12V_ACC = 33
_GG_ENABLE_5V_ACC = 34
_GG_ENABLE_MOTOR_PWR = 35

# Motor drive strength (0-255)
_GG_MOTOR_MAG_START = 36

# Motor Direction
_GG_MOTOR_DIR_START = 38

# Error detection, errors also trigger Error LED, in some cases error flags can be cleared with a write
# to these registers, Should we?
_GG_MOTOR_ERROR_STATE = 40
_GG_SYSTEM_ERROR_STATE = 41

V_ZEN = (10.1)

def read_high_low_data(bus, address):
    """Fetches and combines data stored across two bytes"""
    high_value = bus.read_byte_data(_GG_I2C_ADDR, address)
    low_value = bus.read_byte_data(_GG_I2C_ADDR, address + 1)
    return low_value + (high_value << 8)

def write_high_low_data(bus, address, data):
    value = int(data)
    bus.write_byte_data(_GG_I2C_ADDR, address, value >> 8)
    bus.write_byte_data(_GG_I2C_ADDR, address + 1, value & 0xff)


class GreenGiantInternal():
    """Intertions for use with the user
    not intended to be part of the robot API"""

    def __init__(self, bus):
        self._bus = bus
        self._version = self.get_version()
        self.enabled_motors = False
        self.set_motor_power(self.enabled_motors)

    def enable_motors(self,new_state):
        if self._version < 10:
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, int(new_state))
            self.enabled_motors = new_state
            # Up to and including GreenGiant v3 there is no way of reading the state of 
            # the 12v rail. This is more than a software change because to be useful
            # we would have to monitor the output of the high side switch
        else:
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, int(new_state))

    def set_motor_power(self, new_state):
        """Enable the 12v power to the motors, should only need doing at start of day
        Users should not really ever need to look at this for Pi_Low
        For GG the 12v accessory power is also switched this way"""
        if self._version < 10:
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, int(new_state))
            self.enabled_motors = new_state
            # Up to and including GreenGiant v3 there is no way of reading the state of 
            # the 12v rail. This is more than a software change because to be useful
            # we would have to monitor the output of the high side switch
        else:
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTOR_PWR, int(new_state))

    def set_12v_acc_power(self, new_state):
       if self._version < 10:
            # for GG, this is the same as the motor power above
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, int(new_state))
            self.enabled_motors = new_state
            # Up to and including GreenGiant v3 there is no way of reading the state of 
            # the 12v rail. This is more than a software change because to be useful
            # we would have to monitor the output of the high side switch
       else:
            self.enabled_motors = new_state # Bug in PiLow firmware v11 and below
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_12V_ACC, int(new_state))
    
    def get_12v_acc_power(self):
        if self._version <= 12:
            # for GG, this is the same as the motor power above
            return self.enabled_motors
            # Up to and including GreenGiant v3 there is no way of reading the state of 
            # the 12v rail. This is more than a software change because to be useful
            # we would have to monitor the output of the high side switch
        else:
            return self._bus.read_byte_data(_GG_I2C_ADDR, _GG_ENABLE_12V_ACC)

    def set_5v_acc_power(self, new_state):
        if self._version >= 10:
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_5V_ACC, int(new_state))
        else:
            # for GG versions 5v power is always enabled
            raise IOError(f"Attempted to set 5v power to {new_state} on an unsupported BrainBox.")

    def get_5v_acc_power(self):
        if self._version >= 10:
            return bool(self._bus.read_byte_data(_GG_I2C_ADDR, _GG_ENABLE_5V_ACC))
        else:
            # for GG versions 5v power is always enabled
            raise IOError(f"Attempted to get 5v power on an unsupported BrainBox.")


    def set_user_led(self, on):
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_USER_LED, int(on))

    def get_version(self):
        return self._bus.read_byte_data(_GG_I2C_ADDR, _GG_VERSION)

    def get_battery_voltage(self):
        # Firmware version 12 and later reports voltage differently
        if self._version < 12:
            # both GG and PiLow use a 1/3 divider and a 4.096v reference giving a max readable voltage of ~12.3v
            return read_high_low_data(self._bus, _GG_BATTERY_V_H) * _GG_BATTERY_MAX_READING / _GG_BATTERY_ADC_MAX
        else:
            # Hardware change for Cambridge brains in Nov 2025 to use a zener diode
            return ((read_high_low_data(self._bus, _GG_BATTERY_V_H) / 65535) * 4.096) + V_ZEN


    def get_fvr_reading(self):
        """Return the fixed voltage reading. The number read here is sampling the 4.096v reference using the VCC rail (GG only)
        This magic number back calculates what the voltage is on the VCC rail for more accurate voltage readings.
        """
        return _GG_FVR_VOLTS * _GG_BATTERY_ADC_MAX / read_high_low_data(self._bus, _GG_FVR_H)


class GreenGiantGPIOPin():
    def __init__(self, pin_list, bus, version, adc_max, gpio_base_address, pwm_base_address, analog_base_address):
        self._pin_list = pin_list
        self._bus = bus
        if gpio_base_address is None:
            # no way to configure the pin type, it must be hard coded as a PWM_SERVO
            self._mode = PWM_SERVO
        else:
            self._mode = INPUT
        self._adc_max = adc_max
        self._digital_read_modes = (INPUT, INPUT_PULLUP, OUTPUT) ## why not hard coded?
        self._analog_read_modes = (INPUT_ANALOG, PWM_SERVO)
        self._version = version
        self._gpio_base = gpio_base_address
        self._pwm_base = pwm_base_address
        self._analog_base = analog_base_address
        self._set_to = 0

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        if self._gpio_base is not None:
            self._mode = mode
            if self._pin_list is not None:
                """Update the mode of all of the pins.
                Due to a bug in the GG software all of the pins modes need to be updated
                in order
                `_pin_list.update_mode` triggers all of the pins to be re-updated with
                the new settings
                """
                self._pin_list.update_modes()
            else:
                self.update_mode()
        else:
            # this should only happen if we mode set the on the GreenGiant, allow only PWM_SERVO
            if mode is INPUT:
                # no input setting, but we can set the servo to neutral 
                self.pwm(0);
            elif mode is not PWM_SERVO:
                raise IOError(f"Attempt to set PWM only pin as {self._mode}")


    def update_mode(self):
        """Writes a mode update (for this pin only) to the I2C bus"""
        if self._gpio_base is not None:
            mask = _GG_GPIO_MASKS[self._mode]
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_CONTROL_START + self._gpio_base, mask)
    @property
    def digital(self):
        if self._gpio_base is not None:
            if self._mode not in self._digital_read_modes:
                raise IOError(f"Digital read attempted on pin configured as {self._mode} "
                              f"but this requires mode set to one of {self._digital_read_modes}")
            return bool(self._bus.read_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._gpio_base))
        else:
            raise IOError(f"Attempt to read PWM only pin")

    @digital.setter
    def digital(self, value):
        if self._gpio_base is not None:
            if self._mode is not OUTPUT:
                raise IOError(f"Digital write attempted on pin configured as {self._mode} "
                              f"but this requires mode set to {OUTPUT} ")
            self._bus.write_byte_data(_GG_I2C_ADDR, _GG_DIGITAL_START + self._gpio_base, int(value))
            self._set_to = bool(value)
            #print(f"Value set to {bool(value)} on address {_GG_DIGITAL_START + self._gpio_base}")
        else:
            raise IOError(f"Attempt to write to a PWM only pin")

    @property
    def analog(self):
        """Reads an analog value from the ADC and converts it to a voltage"""
        if self._gpio_base is not None:
            if self._mode not in self._analog_read_modes:
                raise IOError(f"Analog read attempted on pin configured as {self._mode} "
                              f"but this requires mode set to {self._analog_read_modes} ")

            # We have a 10 bit ADC this is the maximum value we could read from it
            raw_adc_max = 0xFFC0
            raw_adc_value = read_high_low_data(self._bus, self._analog_base)
            return (raw_adc_value / raw_adc_max) * self._adc_max
        else:
            raise IOError(f"Attempt to read PWM only pin")

    @property
    def pwm(self):
        if self._version <= 3:
            # create an illusion for the GreenGiant, we cant read the state of an output
            return self._set_to
        else:
            if self._mode is not PWM_SERVO:
                raise IOError(f"Attempt to read PWM property from pin configured as {self._mode}")
            raw = read_high_low_data(self._bus, self._analog_base)
            if self._version < 10:
               return ((raw - _GG_GG_PWM_CENTER) / _GG_GG_PWM_PERCENT_HALF_RANGE) * 100
            else:
               return ((raw - _GG_PiLow_PWM_CENTER) / _GG_PiLow_PWM_PERCENT_HALF_RANGE) * 100
    @pwm.setter
    def pwm(self, percent):
        if self._pwm_base is not None:
            if self._mode is not PWM_SERVO:
                raise IOError(f"Attempt to set PWM value on pin configured as {self._mode}")

            """ sets the PWM output value for use with RC servos and external motor controllers """
            self._set_to = percent
            if self._version < 10:
                value = _GG_GG_PWM_CENTER + ((percent * _GG_GG_PWM_PERCENT_HALF_RANGE) / _GG_GG_PWM_HALF_RANGE)
                value = clamp(value, _GG_GG_PWM_MIN, _GG_GG_PWM_MAX)
            else:
                value = _GG_PiLow_PWM_CENTER + ((percent / _GG_PiLow_PWM_PERCENT_HALF_RANGE) * _GG_PiLow_PWM_HALF_RANGE)
                value = clamp(value, _GG_PiLow_PWM_MIN, _GG_PiLow_PWM_MAX)
            write_high_low_data(self._bus, self._pwm_base, value)
        else:
            raise IOError(f"Attempt set PWM value on GPIO only pin")

    #def __bool__(self):
    #    """Return a bool if in a digital mode or a float if in an analogue"""
    #    if self._gpio_base is not None:
    #        if self._mode in self._digital_read_modes:
    #            return self.digital
    #
    #        raise ValueError(f"Tried to evaluate GPIO {self._index} as a True/False "
    #                         f"but current mode:{self._mode} is not in {self._digital_read_modes}")
    #    else:
    #        raise IOError(f"Attempt use value of a PWM only pin")

    def __setitem__(self, value):
        if self._mode is PWM_SERVO:
            self.pwm = value
        elif self._mode is OUTPUT:
            self.digital = value
        else:
            raise IOError(f"Attempt to write to a pin configured for input")

    def __getitem__(self):
        if self._mode is PWM_SERVO:
            return self.pwm
        elif self._mode in self._digital_read_modes:
            return self.digital
        elif self._mode in self._analog_read_modes:
            return self.analog



class GreenGiantGPIOPinList():
    """A list of pins indexed from 1 (GG) or 0 (later) """

    def __init__(self, bus, version, adc_max, gpio_base_address, pwm_base_address):
        if version <= 3:
            # we need to update the pins in order to work around a GreenGiant bug
            pinlist = self
        else:
            pinlist = None

        if gpio_base_address is not None:
            if pwm_base_address is not None:
                self._list = [GreenGiantGPIOPin(pinlist, bus, version, adc_max, gpio_base_address + i, 
                              pwm_base_address + (2*i), gpio_base_address + (2*i))
                      for i in range(4)]
            else:
                self._list = [GreenGiantGPIOPin(pinlist, bus, version, adc_max, gpio_base_address +i, 
                              None , gpio_base_address + (2*i))
                      for i in range(4)]
        else:
            if pwm_base_address is not None:
                self._list = [GreenGiantGPIOPin(pinlist, bus, version, adc_max, None, 
                              pwm_base_address + (2*i), None )
                      for i in range(4)]
            else:
                raise IOError(f"No base addresses, what are we even doing here")
        self._version = version
        self.update_modes()  # Make sure the state of the GG matches the state
                             # which we have assumed (INPUT) when creating
                             # GreenGiantGPIOPin's

    def __getitem__(self, index):
        if self._version < 10:
            return self._list[_decrement_pin_index(index)]
        else:
            return self._list[index]

    def __setitem__(self, index, value):
        if self._version < 10:
            internal_index = _decrement_pin_index(index)
        else:
            internal_index = index
        self._list[internal_index].__setitem__(value)

    def update_modes(self):
        """All of the modes must be updated in order due to a bug in the GG
        This function provides a function aware of the state of all of the pins
        which can update them on the GG.
        """
        for pin in self._list:
            pin.update_mode()

    def off(self):
        for pin in self._list:
            pin.mode = INPUT
"""
class GreenGiantPWM():
    ""An object implementing a descriptor protocol to control the servos for Green Giant only
    PWM is combined into GPIO for the PiLow
    ""

    def __init__(self, bus, version):
        self._bus = bus
        self._version = version
    def __getitem__(self, index):
        if self._version < 10:
            index = _decrement_pin_index(index)

        command = _GG_PWM_START + (index * 2)
        # TODO - Use a function for this?
        high = self._bus.read_byte_data(_GG_I2C_ADDR, command)
        low = self._bus.read_byte_data(_GG_I2C_ADDR, command + 1)
        value = low + (high << 8)

        if self._version < 10:
            return (value - _GG_GG_PWM_CENTER) * 100 / _GG_GG_PWM_PERCENT_HALF_RANGE
        else:
            return (value - _GG_PiLow_PWM_CENTER) * 100 / _GG_PiLow_PWM_PERCENT_HALF_RANGE

    def __setitem__(self, index, percent):
        if self._version < 10:
            index = _decrement_pin_index(index)
        command = _GG_PWM_START + (index * 2)
        if self._version < 10:
            value = _GG_GG_PWM_CENTER + (percent / 100 * _GG_GG_PWM_PERCENT_HALF_RANGE)
            value = clamp(value, _GG_GG_PWM_MIN, _GG_GG_PWM_MAX)
        else:
            value = _GG_PiLow_PWM_CENTER + (percent / 100 * _GG_PiLow_PWM_PERCENT_HALF_RANGE)
            value = clamp(value, _GG_PiLow_PWM_MIN, _GG_PiLow_PWM_MAX)
        value = int(value)

        low = value & 0xFF
        high = value >> 8

        self._bus.write_byte_data(_GG_I2C_ADDR, command, high)
        self._bus.write_byte_data(_GG_I2C_ADDR, command + 1, low)

    def off(self):
        for i in range(4):
            if self._version < 10:
                self.__setitem__(i + 1, 0)
            else:
                self.__setitem__(i, 0)
"""
_SYSTEM_VOLTAGE = 12
_MAX_MOTOR_PWM_VALUE = 0xff

class GreenGiantMotors():
    def __init__(self, bus, max_motor_voltage):
        """The interface to the PiLow cover
        max_motor_voltage - The motors power will be scaled so that the max power
                            delivered equals the power that this voltage
        """
        self._bus=bus
        if not (0 <= max_motor_voltage <= _SYSTEM_VOLTAGE):
            raise ValueError("max_motor_voltage must satisfy 0 <= "
                             "max_motor_voltage <= 12 but instead is "
                             f"{max_motor_voltage}")

        # because we care about heating effects in the motors, we have to scale by
        # the square of the ratio
        self.power_scaling_factor = (
            max_motor_voltage / _SYSTEM_VOLTAGE) ** 2

        # should we set up the state of 12v power and enable here?
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, 0) # disable the motor controller

    def enable_motors(self, value):
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, value)


    def __getitem__(self, index):
        """Returns the current PWM value in RC units. Adds a sign to represent"""
        if index not in (0,1):
            raise IndexError(
                f"motor index must be in (0,1) but instead got {index}")
        hex_mag = self._bus.read_byte_data(_GG_I2C_ADDR, _GG_MOTOR_A_MAG + index)

        return hex_mag * (100.0 / 256.0) * self.power_scaling_factor

    def __setitem__(self, index, percent):
        """Clamps input value, converts from percentage to wiring pi format and
        sets a PWM format"""
        if index not in (0,1):
            raise IndexError(
                f"motor index must be in (0,1) but instead got {index}")

        direction = (percent < 0)
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_MOTOR_DIR_START + index, direction)

        # Scale such that 50% with a motor limit of 6V is really 3V
        scaled_value = clamp(abs(percent) * self.power_scaling_factor * (256 / 100), 0, 255)
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_MOTOR_MAG_START + index, int(scaled_value))

    def stop(self):
        """Turns motors off"""
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_ENABLE_MOTORS, 0)
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_MOTOR_MAG_START, 0)
        self._bus.write_byte_data(_GG_I2C_ADDR, _GG_MOTOR_MAG_START + 1, 0)

