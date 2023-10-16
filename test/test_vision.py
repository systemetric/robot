import unittest
import time
import numpy
from datetime import datetime

import cv2

import robot

image_mosaic = "/home/pi/robot/test/mosaic.png"
image_arc = "/home/pi/robot/test/arc.png"

mock_fl = (607.6669874845361, 607.6669874845361)


class MockCamera(robot.vision.Camera):
    """A Mock camera for returning precaptured images"""
    def __init__(self):
        self._res = (640, 480)
        self._mock_fl_lut = {
            self.res: mock_fl
        }
        self._update_camera_params(self._mock_fl_lut)
        self.colour_frame = cv2.imread(image_mosaic)

    def set_mock_image(self, path, scale=None):
        """Load image located at `path` as the mock image

        scale - the scale factor to scale the image by
        """
        self.colour_frame = cv2.imread(path)
        height, width, depth = numpy.shape(self.colour_frame)
        self._res = (height, width)
        if scale is not None:
            new_size = (height * scale, width * scale)
            cv2.resize(self.colour_frame, new_size, interpolation=cv2.INTER_AREA)
            self._res = new_size
        self._mock_fl_lut[self._res] = mock_fl
        self._update_camera_params(self._mock_fl_lut)


    @property
    def res(self):
        return self._res

    @res.setter
    def res(self, new_res):
        self._res = new_res

    def capture(self):
        capture_time = datetime.now()
        grey_frame = cv2.cvtColor(self.colour_frame, cv2.COLOR_BGR2GRAY)
        return robot.vision.Capture(grey_frame=grey_frame,
                                    colour_frame=self.colour_frame,
                                    colour_type="RGB",
                                    time=capture_time)

    def close(self):
        print("MockCamera close called")


class RobotVision(unittest.TestCase):
    """Tests the vision module behaves as we expect"""
    @classmethod
    def setUpClass(cls):
        cls.r = robot.Robot(camera=MockCamera)

    @classmethod
    def tearDownClass(cls):
        if cls.r is not None:
            del cls.r
            cls.r = None

    def test_marker_numbering(self):
        """Check the boundaries of the marker numbering is what we expect

        Should help prevent off by one errors.
        """
        self.r.camera.set_mock_image(image_mosaic, scale=10)

        # (marker_code, marker_type)
        marker_numbering_tests = [
            (0, robot.MARKER_ARENA),
            (32, robot.MARKER_ARENA),
            (33, robot.MARKER_TOKEN),
            (40, robot.MARKER_TOKEN),
            (41, robot.MARKER_DEFAULT)
        ]

        markers = self.r.see()
        print(f"markers {markers}")
        # marker_codes = [m.code for m in markers]
        # print(f"got {marker_codes}")
        # for marker in markers:
        #     with self.subTest(marker=marker):
        #         expected_type = None
        #         for code, type_ in marker_numbering_tests:
        #             print(f"test_code {code}")
        #             print(f"marker.code got code {marker.code}")
        #             if code == marker.code:
        #                 expected_type = type_
        #                 break
        #         self.assertEqual(marker.type, expected_type)

    def test_bearings_from_real_image(self):
        """test `marker.bear.y` increases left to right in real image

        These numbers were manually checked.
        """
        self.r.camera.set_mock_image(image_arc)

        markers = self.r.see()

        tests = [
            (bear_y, rot_y, marker_code)
            for (marker_code, bear_y, rot_y) in (
                (22, 5.9020841573582175, 6.588315444505456),
                (24, 12.037433933440928, 23.642168800862297),
                (58, 18.891952258734502, 53.18260687019854),
                (85, -13.968911862136904, -44.657545026570745),
                (144, -2.592376356827769, -27.045341210472248),
                (198, 26.18739331265824, 60.27103778528233),


            )
        ]
        for expected_bear_y, expected_rot_y, marker_code in tests:
            with self.subTest(expected_bear_y=expected_bear_y,
                              expected_rot_y=expected_rot_y,
                              marker_code=marker_code):
                for m in [m for m in markers if m.code == marker_code]:
                    self.assertAlmostEqual(m.bear.y, expected_bear_y, delta=0.5)
                    self.assertAlmostEqual(m.rot.y, expected_rot_y, delta=0.5)
