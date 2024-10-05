"""An interface to the cyctron motor board. A gpio pin is used for each motor
to give direction and has a PWM signal at 100Hz giving infomation about voltage
to apply
"""
import wiringpi as wp
from robot.greengiant import clamp

_MAX_OUTPUT_VOLTAGE = 12

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


class CytronBoard():
    def __init__(self, max_motor_voltage):
        """The interface to the CytronBoard
        max_motor_voltage - The motors will be scaled so that this is the maxium
                            average voltage the Cyctron will output
        """
        # Set up Wiring Pi following the wiring pi numbering scheme
        if wp.wiringPiSetup() != 0:
            raise RuntimeError("Failed to init Wiring Pi")

        if not (0 <= max_motor_voltage <= 12):
            raise ValueError("max_motor_voltage must satisfy 0 <= "
                             "max_motor_voltage <= 12 but instead is "
                             f"{max_motor_voltage}")

        # because we care about heating effects in the motors, we have to scale by
        # the square of the ratio
        self.power_scaling_factor = (
            max_motor_voltage / _MAX_OUTPUT_VOLTAGE) ** 2

        self._percentages = [0, 0]
        self._dir = [_DIR_PIN_1, _DIR_PIN_2]
        self._pwm_pins = [_PWM_PIN_1, _PWM_PIN_2]

        wp.pinMode(_DIR_PIN_1, _WP_OUT)
        wp.pinMode(_DIR_PIN_2, _WP_OUT)
        wp.pinMode(_PWM_PIN_1, _WP_PWM)
        wp.pinMode(_PWM_PIN_2, _WP_PWM)

        wp.pwmSetClock(3000)
        for pin in self._pwm_pins:
            wp.pwmWrite(pin, 0)

    def __getitem__(self, index):
        """Returns the current PWM value in RC units. Adds a sign to represent"""
        if index not in (1, 2):
            raise IndexError(
                f"motor index must be in (1,2) but instead got {index}")

        index -= 1
        return self._percentages[index]

    def __setitem__(self, index, percent):
        """Clamps input value, converts from percentage to wiring pi format and
        sets a PWM format"""
        if index not in (1, 2):
            raise IndexError(
                f"motor index must be in (1,2) but instead got {index}")

        index -= 1
        percent = clamp(percent, -100, 100)
        self._percentages[index] = percent

        direction = (percent < 0)
        wp.digitalWrite(self._dir[index], direction)

        # Scale such that 50% with a motor limit of 6V is really 3V
        scaled_value = abs(percent) * self.power_scaling_factor
        wp_value = rc_to_wp_pwm(scaled_value)
        wp.pwmWrite(self._pwm_pins[index], wp_value)

    def stop(self):
        """Turns motors off"""
        for pwm_pin, percentage in zip(self._pwm_pins, self._percentages):
            wp.pwmWrite(pwm_pin, 0)
            percentage = 0
