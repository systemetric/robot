import importlib.util

has_rpi = importlib.util.find_spec("RPi") is not None

if not has_rpi:
    import sys
    import fake_rpi
    sys.modules["RPi"] = fake_rpi.RPi
    sys.modules["RPi.GPIO"] = fake_rpi.RPi.GPIO
    sys.modules["smbus2"] = fake_rpi.smbus

import sys

# greengiant imports for users
from robocon.brain.greengiant import OUTPUT, INPUT, INPUT_ANALOG, INPUT_PULLUP, PWM_SERVO
from robocon.brain.io import IO

__all__ = (
    "IO",
    "OUTPUT",
    "INPUT",
    "INPUT_ANALOG",
    "INPUT_PULLUP",
    "PWM_SERVO",
)
