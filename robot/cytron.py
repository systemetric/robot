"""
An interface to the cyctron motor board. A GPIO pin is used for each motor
to give direction and has a PWM signal at 100Hz giving infomation about voltage
to apply
"""

import RPi.GPIO as GPIO
from .greengiant import clamp

_MAX_OUTPUT_VOLTAGE = 12

_PWM_PIN_1 = 12
_PWM_PIN_2 = 13
_DIR_PIN_1 = 26
_DIR_PIN_2 = 24

class CytronBoard:
    def __init__(self, max_motor_voltage):
        """
        The interface to the CytronBoard
        max_motor_voltage - The motors will be scaled so that this is the maxium
                            average voltage the Cytron will output
        """
        if not (0 <= max_motor_voltage <= 12):
            raise ValueError("max_motor_voltage must satisfy 0 <= "
                             "max_motor_voltage <= 12 but instead is "
                             f"{max_motor_voltage}")

        # because we care about heating effects in the motors, we have to scale by
        # the square of the ratio
        self.power_scaling_factor = (
            max_motor_voltage / _MAX_OUTPUT_VOLTAGE) ** 2

        self._dir = [
            (GPIO.LOW, _DIR_PIN_1),
            (GPIO.LOW, _DIR_PIN_2),
        ]
        self._pwm = [
            (0, GPIO.PWM(_PWM_PIN_1, 100)),
            (0, GPIO.PWM(_PWM_PIN_2, 100)),
        ]

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(_DIR_PIN_1, GPIO.OUT)
        GPIO.setup(_DIR_PIN_2, GPIO.OUT)
        GPIO.setup(_PWM_PIN_1, GPIO.OUT)
        GPIO.setup(_PWM_PIN_2, GPIO.OUT)

    def __getitem__(self, index):
        """Returns current motor PWM value as a percentage"""
        if index not in (0, 1):
            raise IndexError(
                f"Motor index must be in (0,1) but instead got {index}")

        return self._pwm[index][0]

    def __setitem__(self, index, percent):
        """Set current motor PWM percentage value"""
        if index not in (0, 1):
            raise IndexError(
                f"Motor index must be in (0,1) but instead got {index}")

        if percent < 0:
            self._dir[index][0] = GPIO.LOW
        else:
            self._dir[index][0] = GPIO.HIGH

        GPIO.output(self._dir[index][1], self._dir[index][0])

        percent = clamp(percent, -100, 100)
        self._pwm[index][0] = percent

        percent = abs(percent) * self.power_scaling_factor
        self._pwm[index][1].start(percent)

    def stop(self):
        """Turns motors off"""
        for i in range(len(self._pwm)):
            self._pwm[i][0] = 0
            self._pwm[i][1].start(0)
