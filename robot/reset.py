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

from wardog.client import *

def reset():
    """Resets the robot components to their default state.
    Used by Shepherd when the Stop button is pressed.
    """
    wardog = WarDogClient("robot-reset")
    wardog.send_message(WarDogRequest("init_hwc"))
    
    #if version < 10:
    #    c.CytronBoard(1).stop()
    #    gg.GreenGiantGPIOPinList(bus, version, 5, None, gg._GG_SERVO_PWM_BASE)
    #    gg.GreenGiantGPIOPinList(bus, version, 5, gg._GG_GPIO_GPIO_BASE, None)
    #else:
    #    gg.GreenGiantMotors(bus, 1).stop()
    #    gg.GreenGiantGPIOPinList(bus, version, 5, gg._GG_SERVO_GPIO_BASE, gg._GG_SERVO_PWM_BASE).off()
    #    gg.GreenGiantGPIOPinList(bus, version, 5, gg._GG_GPIO_GPIO_BASE, gg._GG_GPIO_PWM_BASE).off()
