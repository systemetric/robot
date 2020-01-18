from smbus2 import SMBus
import cytron as c
import greengiant as gg

"""
Resets the robot components to their default state.
Used by Shepherd when the Stop button is pressed.
"""
def reset():
    bus = SMBus(1)

    c.CytronBoard(c.DEFAULT_MOTOR_CLAMP).stop()
    gg.GreenGiantPWM(bus).off()
    for i in range(4):
        gg.GreenGiantGPIOPin(bus, i, 4.096).mode = gg.INPUT
    internal = gg.GreenGiantInternal(bus)
    internal.set_12v(False)
    internal.set_status_led(True)

    bus.close()
