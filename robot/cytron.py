import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_CYTRON_GPIO_DIR_1 = 26
_CYTRON_GPIO_DIR_2 = 24
_CYTRON_GPIO_PWM_1 = 12
_CYTRON_GPIO_PWM_2 = 13


class CytronBoard(object):
    def __init__(self):
        self.safety_override = False

        GPIO.setup(_CYTRON_GPIO_DIR_1, GPIO.OUT)
        GPIO.setup(_CYTRON_GPIO_DIR_2, GPIO.OUT)
        GPIO.setup(_CYTRON_GPIO_PWM_1, GPIO.OUT)
        GPIO.setup(_CYTRON_GPIO_PWM_2, GPIO.OUT)

        sleep(0.5)

        self._dir_value = [GPIO.LOW, GPIO.LOW]
        self._dir = [
            _CYTRON_GPIO_DIR_1,
            _CYTRON_GPIO_DIR_2
        ]

        self._pwm_value = [0, 0]
        self._pwm = [
            GPIO.PWM(_CYTRON_GPIO_PWM_1, 100),
            GPIO.PWM(_CYTRON_GPIO_PWM_2, 100)
        ]

    def __getitem__(self, index):
        if not (1 <= index <= 2):
            raise IndexError("motor index must be 1 or 2")
        index -= 1

        value = self._pwm_value[index]
        if self._dir_value[index] == GPIO.HIGH:
            value = -value
        return value

    def __setitem__(self, index, percent):
        if not (1 <= index <= 2):
            raise IndexError("motor index must be 1 or 2")
        index -= 1

        max_value = 100 if self.safety_override else 25

        abs_value = abs(percent)
        if abs_value == percent:
            self._dir_value[index] = GPIO.LOW
        else:
            self._dir_value[index] = GPIO.HIGH

        GPIO.output(self._dir[index], self._dir_value[index])

        value = abs_value * max_value / 100

        self._pwm_value[index] = abs_value
        self._pwm[index].start(self._pwm_value[index])

    def stop(self):
        for i in range(len(self._pwm)):
            self._pwm[i].start(0)
            self._pwm_value[i] = 0
