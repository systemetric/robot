from smbus2 import SMBus
import robot.cytron as c
import robot.greengiant as gg

def reset():
    """
    Resets the robot components to their default state.
    Used by Shepherd when the Stop button is pressed.
    """
    bus = SMBus(1)

    c.CytronBoard().stop()
    gg.GreenGiantPWM(bus).off()
    for i in range(4):
        gg.GreenGiantGPIOPin(bus, i, 4.096).mode = gg.INPUT
    internal = gg.GreenGiantInternal(bus)
    internal.set_12v(False)
    internal.set_status_led(True)

    bus.close()
