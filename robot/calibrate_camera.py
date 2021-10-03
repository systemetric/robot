"""A script for getting the focal length LUTS for a camera
It losely follows the ideas of a PD controller combined with a NM gradient
descent algorithm.
Usage:
import robot.calibrate_camera
"""
import pprint

import robot

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
    if result == 0:
        result = 1
    elif result > 6:
        result = 6
    return result


R = robot.Robot()
_result = {}

for res in R.camera.focal_lengths.copy():
    print(f"Checking res {res}")
    R.camera.res = res
    pprint.pprint(R.camera.focal_lengths)

    ERROR_ = THRESHOLD + 1.0
    previous_error = ERROR_
    while abs(ERROR_) > THRESHOLD:
        value = R.camera.focal_lengths[res][0]
        p = ERROR_ * KP
        d = (previous_error - ERROR_) * KD
        value += p + d

        R.camera.focal_lengths[res] = (value, value)
        R.camera._update_camera_params()

        reading_counts = _get_reading_number(ERROR_)

        dists = [_get_reading() for _ in range(reading_counts)]
        average_dist = (sum(dists))/reading_counts

        previous_error = ERROR_
        ERROR_ = TARGET - average_dist

        print(f"Tried: {value} got dist {average_dist} error: {ERROR_}")
        print(f"    Max: {max(dists)} min: {min(dists)}"
              " range: {max(dists) - min(dists)}")
        print(f"    P = {ERROR_ * KP} reading_counts {reading_counts}")

    _result[res] = (value, value)

pprint.pprint(_result)
