"""A script for getting the focal length luts for a camera
It losely follows the ideas of a PD controller combinded with a NM gradient
descent algo.
Usage:
import robot.calibrate_camera
"""
import robot
import math
import pprint

TARGET = 3.0
THRESHOLD = 0.01
KP = 100
KD = 5
K_READING_COUNTS = 0.5


def _get_reading():
    while True:
        try:
            return R.see()[0].dist
        except IndexError:
            print("saw nothing")


def _get_reading_number(error):
    result = int(K_READING_COUNTS / error)
    result = abs(result)
    if result is 0:
        result = 1
    elif result > 6:
        result = 6
    return result


R = robot.Robot()
result = {}

for res in R.camera.focal_lengths.copy():
    print("Checking res {}".format(res))
    R.camera.res = res
    pprint.pprint(R.camera.focal_lengths)

    error = THRESHOLD + 1.0
    previous_error = error
    while abs(error) > THRESHOLD:
        value = R.camera.focal_lengths[res][0]
        p = error * KP
        d = (previous_error - error) * KD
        value += p + d

        R.camera.focal_lengths[res] = (value, value)
        R.camera._update_camera_params()

        reading_counts = get_reading_number(error)

        dists = [get_reading() for _ in range(reading_counts)]
        average_dist = (sum(dists))/reading_counts

        previous_error = error
        error = TARGET - average_dist

        print("Tried: {} got dist {} error: {}".format(
            value, average_dist, error))
        print("    Max: {} min: {} range: {}".format(
            max(dists), min(dists), max(dists) - min(dists)))
        print("    P = {} reading_counts {}".format(error * KP, reading_counts))

    result[res] = (value, value)

pprint.pprint(result)
