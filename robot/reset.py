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
    version = gg.GreenGiantInternal(bus).get_version()

    if version < 10:
        c.CytronBoard(1).stop()
        gg.GreenGiantGPIOPinList(bus, version, 5, None, gg._GG_SERVO_PWM_BASE)
        gg.GreenGiantGPIOPinList(bus, version, 5, gg._GG_GPIO_GPIO_BASE, None)
    else:
        gg.GreenGiantMotors(bus, 1).stop()
        gg.GreenGiantGPIOPinList(bus, version, 5, gg._GG_SERVO_GPIO_BASE, gg._GG_SERVO_PWM_BASE).off()
        gg.GreenGiantGPIOPinList(bus, version, 5, gg._GG_GPIO_GPIO_BASE, gg._GG_GPIO_PWM_BASE).off()

    # probably should wrap this all up in a .off()
    internal = gg.GreenGiantInternal(bus)
    internal.enable_motors(False)
    #internal.set_motor_power(False)
    internal.set_12v_acc_power(False)    # Not sure, should this be controlled by user?
    internal.set_5v_acc_power(False)
    internal.set_user_led(False)

    bus.close()
