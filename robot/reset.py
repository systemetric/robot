"""Allows shepherd to reset the hardware state from outside of the robot object
This is indirect mutation of robot state which will not be captured in the robot
object.

This is okay because shepherd completely owns the users code.
If it wants reset the hardware state then it doesn't matter what the users code
has done or wants to do; usercode yields to shepherd always.

A nicer solution using class methods:
https://stackoverflow.com/a/45799209/5006710
"""
from smbus2 import SMBus
import robot.cytron as c
import robot.greengiant as gg

def reset():
    """Resets the robot components to their default state.
    Used by Shepherd when the Stop button is pressed.
    """
    bus = SMBus(1)

    c.CytronBoard(c.DEFAULT_MOTOR_CLAMP).stop()
    gg.GreenGiantPWM(bus).off()
    for i in range(4):
        gg.GreenGiantGPIOPin(bus, i, 4.096).mode = gg.INPUT
    internal = gg.GreenGiantInternal(bus)
    internal.set_12v(False)
    internal.set_status_led(True)

    bus.close()
