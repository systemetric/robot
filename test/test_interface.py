"""Test the robot module
For speed all this does it checks that the software interface is consistent and
doesn't raise runtime exceptions. All interactions with the hardware need to
have manual tests
"""
import unittest

import robot


MOTOR_TESTS = [
    (test_value * sign, expected_value * sign, motor)
    for motor in (1, 2)
    for sign in (1, -1)
    for (test_value, expected_value) in (
    (100, 100),    # Test max
    (100.0, 100),  # Test max float
    (125, 100),    # Test above max
    (72, 72),
    (15, 15),
    (32.342, 32.342),
    (0, 0),
)]


class RobotInterface(unittest.TestCase):
    """The interface behaves as expected; software only"""

    @classmethod
    def setUpClass(cls):
        cls.r = robot.Robot()

    @classmethod
    def tearDownClass(cls):
        del cls.r

    def test_see(self):
        """R.see() returns a list"""
        markers = self.r.see()
        self.assertIsInstance(markers, list)

    def test_set_res(self):
        """We can set the res #TODO reference a capture to make sure that the res was set right"""
        test_res = (10, 10)

        starting_res = self.r.res
        self.r.res = test_res
        self.assertEqual(self.r.res, test_res)
        self.r.res = starting_res

    def test_set_motors(self):
        """Motors can be set to a value and can be queried for that value"""
        for value, expected, motor in MOTOR_TESTS:
            with self.subTest(value=value, expected=expected, motor=motor):
                self.r.motors[motor] = value
                self.assertEqual(self.r.motors[motor], expected)
                self.r.motors[motor] = 0  # Reset the motor state

    @unittest.expectedFailure
    def test_servos(self):
        """Servos can be set to a value and can be queried for that value"""
        servo_tests = [
            (test_value * sign, expected_value, servo)
            for servo in (1,2,3,4)
            for sign in (1, -1)
            for (test_value, expected_value) in (
                (50, 50),               # Test integer
                (34.23423, 34.23423),   # Test float
                (110, 110),
                (1000, 1000),
                (0, 0)
            )
        ]
        for value, expected, servo in servo_tests:
            with self.subTest(value=value, expected=expected, servo=servo):
                self.r.servos[servo] = value
                self.assertEqual(self.r.servos[servo], expected)
                self.r.servos[servo] = 0  # Reset the motor state

    @unittest.expectedFailure
    def test_gpio_output_set_and_read(self):
        """GPIOs out digit'als can be set and read"""
        # TODO This isn't' python3 compatiable (probally need to unit test everything)
        gpio_digital_out_tests = [
            (state, pin)
            for state in (True, False)
            for pin in range(0, 4)
        ]
        for state, pin in gpio_digital_out_tests:
            with self.subTest(state=state, pin=pin):
                print(type(self.r.gpio[pin]))
                self.r.gpio[pin].mode = robot.OUTPUT
                self.r.gpio[pin].digital = state
                self.assertEqual(self.r.gpio[pin].digital, state)


class RobotInit(unittest.TestCase):
    """No runtime errors from the `Robot` `kwargs`"""
    def test_motor_max_in_range(self):
        """Robot can have a custom max_motor_voltage less than 12V
        TODO remove duplicated code from the motor interface test
        """
        R = robot.Robot(max_motor_voltage=6)
        for value, expected, motor in MOTOR_TESTS:
            with self.subTest(value=value, expected=expected, motor=motor):
                R.motors[motor] = value
                self.assertEqual(R.motors[motor], expected)
                R.motors[motor] = 0  # Reset the motor state

    def test_motor_max_float(self):
        """Robot can have a custom max_motor_voltage less than 12V and is float
        """
        R = robot.Robot(max_motor_voltage=7.2)
        for value, expected, motor in MOTOR_TESTS:
            with self.subTest(value=value, expected=expected, motor=motor):
                R.motors[motor] = value
                self.assertEqual(R.motors[motor], expected)
                R.motors[motor] = 0  # Reset the motor state

    def test_motor_max_out_range(self):
        """Robot max_motor_voltage above 12 prevents init"""
        with self.assertRaises(ValueError):
            R = robot.Robot(max_motor_voltage=20)

    def test_use_usb_cam(self):
        """Robot can be init with a USB camera
        TODO raise an error if a USB camera is not plugged in
        """
        R = robot.Robot(camera=robot.RoboConUSBCamera)
        R.see()

    def test_wait_for_start_pauses_initalisation(self):
        """Can move servos before competition starts"""
        R = robot.Robot(wait_for_start=False)
        R.servos[1] = 100
        R.motors[1] = 100
        R.wait_start()
        # Reset state
        R.servos[1] = 0
        R.motors[1] = 0
