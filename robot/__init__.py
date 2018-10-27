from cytron import CytronBoard
from greengiant import GreenGiantGPIOPin, GreenGiantPWM, OUTPUT, INPUT, INPUT_ANALOG, INPUT_PULLUP
import smbus


class Robot(object):
    def __init__(self):
        self._bus = smbus.SMBus(1)

        self.motors = CytronBoard()
        self.gpio = [None]
        for i in range(4):
            self.gpio.append(GreenGiantGPIOPin(self._bus, i))

        self.pwm = GreenGiantPWM(self._bus)

    def stop(self):
        self.motors.stop()


__all__ = [
    "Robot", "OUTPUT", "INPUT", "INPUT_ANALOG", "INPUT_PULLUP"
]
