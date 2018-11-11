from smbus2 import SMBus
import cytron as c
import greengiant as gg


def reset():
    bus = SMBus(1)

    c.CytronBoard().stop()
    gg.GreenGiantPWM(bus).off()
    for i in range(4):
        gg.GreenGiantGPIOPin(bus, i, 4.096).mode = gg.INPUT
    internal = gg.GreenGiantInternal(bus)
    internal.set_12v(False)
    internal.set_status_led(True)

    bus.close()
