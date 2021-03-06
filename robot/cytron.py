"""An interface to the cyctron motor board. A gpio pin is used for each motor
to give direction and has a PWM signal at 100Hz giving infomation about voltage
to apply
"""
import RPi.GPIO as GPIO
from robot.greengiant import clamp

DEFAULT_MOTOR_CLAMP = 25


_PWM_PIN_1 = 26
_PWM_PIN_2 = 23
_DIR_PIN_1 = 25
_DIR_PIN_2 = 5

_WP_OUT = 1
_WP_PWM = 2

# Wiring pi's PWM has range 0-1024 but we want to present a range of 0-100
_WP_PWM_MAX = 1024
_RC_PWM_MAX = 100


def wp_to_rc_pwm(wp_pwm):
    """Convert from wiring pi's numbering to percentages"""
    return (wp_pwm * _RC_PWM_MAX)/_WP_PWM_MAX


def rc_to_wp_pwm(rc_pwm):
    """Convert from percenatges to wiring pi's numbering"""
    return int((rc_pwm * _WP_PWM_MAX)/_RC_PWM_MAX)


# Set up Wiring Pi following the wiring pi numbering scheme
assert wp.wiringPiSetup() == 0


class CytronBoard(object):
    """An object overiding the array set and get methods to provide a nice
    interface for setting and getting pwm values"""
    def __init__(self, max_value):
        self.max_value = max_value

        wp.pinMode(_DIR_PIN_1, _WP_OUT)
        wp.pinMode(_DIR_PIN_2, _WP_OUT)
        wp.pinMode(_PWM_PIN_1, _WP_PWM)
        wp.pinMode(_PWM_PIN_2, _WP_PWM)


        self._dir_value = [False, False]
        self._dir = [
            _DIR_PIN_1,
            _DIR_PIN_2
        ]

        self._pwm_value = [0, 0]
        self._pwm_pins = [
            _PWM_PIN_1,
            _PWM_PIN_2
        ]

        wp.pwmSetClock(3000) # Set the clock to 100Hz????
        [wp.pwmWrite(pin, 0) for pin in self._pwm_pins]

    def __getitem__(self, index):
        if index not in (1, 2):
            raise IndexError(
                "Expected motor index to be 1 or 2 but instead got {}".format(index))

        index -= 1

        value = wp_to_rc_pwm(self._pwm_value[index])
        if self._dir_value[index]:
            value = -value
        return value

    def __setitem__(self, index, percent):
        if index not in (1, 2):
            raise IndexError(
                "Expected motor index to be 1 or 2 but instead got {}".format(index))

        index -= 1

        abs_value = abs(percent)
        if abs_value == percent:
            self._dir_value[index] = False
        else:
            self._dir_value[index] = True

        wp.digitalWrite(self._dir[index], self._dir_value[index])

        # Scale, clamp and then convert to wp units the pwm value
        value = abs_value * self.max_value / 100
        value = clamp(value, 0, self.max_value)
        value = rc_to_wp_pwm(value)

        self._pwm_value[index] = value
        wp.pwmWrite(self._pwm_pins[index], self._pwm_value[index])

    def stop(self):
        """Turns motors off"""
        for i in range(len(self._pwm_pins)):
            wp.pwmWrite(self._pwm_pins[i], 0)
            self._pwm_value[i] = 0

