import smbus
import cytron as c
import greengiant as gg


def reset():
    bus = smbus.SMBus(1)

    c.CytronBoard().stop()
    gg.GreenGiantPWM(bus).off()
    for i in range(4):
        gg.GreenGiantGPIOPin(bus, i).mode = gg.INPUT
    gg.GreenGiantInternal(bus).set_12v(False)

    bus.close()
