"""An interface to the cyctron motor board. A gpio pin is used for each motor
to give direction and has a PWM signal at 100Hz giving infomation about voltage
to apply
"""
import RPi.GPIO as GPIO
from robot.greengiant import clamp


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_CYTRON_GPIO_DIR_1 = 26
_CYTRON_GPIO_DIR_2 = 24
_CYTRON_GPIO_PWM_1 = 12
_CYTRON_GPIO_PWM_2 = 13

_MAX_OUTPUT_VOLTAGE = 12


class CytronBoard():
    def __init__(self, max_motor_voltage):
        """The interface to the CytronBoard
        max_motor_voltage - The motors will be scaled so that this is the maxium
                            average voltage the Cyctron will output
        """
        if not (0 <= max_motor_voltage <= 12):
            raise ValueError("max_motor_voltage must satisfy 0 <= "
                             "max_motor_voltage <= 12 but instead is "
                             f"{max_motor_voltage}")

        self.power_scaling_factor = max_motor_voltage / _MAX_OUTPUT_VOLTAGE

        GPIO.setup(_CYTRON_GPIO_DIR_1, GPIO.OUT)
        GPIO.setup(_CYTRON_GPIO_DIR_2, GPIO.OUT)
        GPIO.setup(_CYTRON_GPIO_PWM_1, GPIO.OUT)
        GPIO.setup(_CYTRON_GPIO_PWM_2, GPIO.OUT)

        self._dir = [_CYTRON_GPIO_DIR_1, _CYTRON_GPIO_DIR_2]

        self._percentages = [0, 0]
        self._pwm = [
            GPIO.PWM(_CYTRON_GPIO_PWM_1, 100),
            GPIO.PWM(_CYTRON_GPIO_PWM_2, 100)
        ]

    def __getitem__(self, index):
        if index not in (1, 2):
            raise IndexError(f"Expected motor index to be 1 or 2 but instead got {index}")

        index -= 1
        return self._percentages[index]

    def __setitem__(self, index, percent):
        if index not in (1, 2):
            raise IndexError(f"Expected motor index to be 1 or 2 but instead got {index}")

        index -= 1
        percent = clamp(percent, -100, 100)
        self._percentages[index] = percent

        direction = GPIO.HIGH if percent < 0 else GPIO.LOW
        GPIO.output(self._dir[index], direction)

        # Scale such that 50% with a limit of 6V is really 3V
        scaled_value = abs(percent) * self.power_scaling_factor
        self._pwm[index].start(scaled_value)

    def stop(self):
        for i in range(len(self._pwm)):
            self._pwm[i].start(0)
            self._percentages[i] = 0
