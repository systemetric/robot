from __future__ import print_function
import robot

R = robot.Robot()

# os.exit(0)

# print(R._internal.get_fvr_reading())

# for i in range(64):
#     print(i, ":", R._internal._bus.read_byte_data(0x8, i))

# for i in range(4):
#     R.gpio[i + 1].mode = robot.INPUT_ANALOG
#
# while True:
#     for i in range(4):
#         print("%.2f" % R.gpio[i + 1].analog, end="\t")
#     print()

# R.see()
#
# R.gpio[1].mode = robot.OUTPUT
# R.gpio[1].analog
#
# R.servos[1] = 100
#
# print "Forward?"
#
# R.motors[1] = SPEED
# R.motors[2] = SPEED
#
# time.sleep(2)
#
# print "Backward?"
#
# R.motors[1] = -SPEED
# R.motors[2] = -SPEED
#
# time.sleep(2)
#
# print "Left?"
#
# R.motors[1] = SPEED
# R.motors[2] = -SPEED
#
# time.sleep(2)
#
# print "Right?"
#
# R.motors[1] = -SPEED
# R.motors[2] = SPEED
#
# time.sleep(2)
#
# R.stop()
