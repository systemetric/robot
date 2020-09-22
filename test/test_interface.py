"""Test the robot module
For speed all this does it checks that the software interface is consistent and
doesn't raise runtime exceptions. All interactions with the hardware need to
have manual tests
"""
import logging
import unittest
import atexit

import robot
import test.harness as harness


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
        """Initalise the robot as a class property so that it doesn't need to
        be reinitialized for everytest."""
        cls.r = robot.Robot()

    @classmethod
    def tearDownClass(cls):
        """Normally this doesn't need to be called as the process gets destroyed
        however here the robot object could be re-initialized for other tests"""
        del cls.r

    def test_see(self):
        """R.see() returns a list of marker objects"""
        markers = self.r.see()
        self.assertIsInstance(markers, list)
        for marker in markers:
            self.assertIsInstance(marker, robot.vision.Marker)

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

    # @will this test fails on my brain, but given my GG
    # also fails the 12v read and jacks doesn't I don't trust my GG reading
    # even though this is a different failure mode
    @unittest.expectedFailure
    def test_servos(self):
        """Servos can be set to a value and can be queried for that value."""
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

    def test_gpio_output_set_and_read(self):
        """GPIOs out digitals can be set and read"""
        gpio_digital_out_tests = [
            (state, pin)
            for state in (True, False)
            for pin in range(1, 5)
        ]
        for state, pin in gpio_digital_out_tests:
            with self.subTest(state=state, pin=pin):
                self.r.gpio[pin].mode = robot.OUTPUT
                self.r.gpio[pin].digital = state
                self.assertEqual(self.r.gpio[pin].digital, state)

    def test_gpio_can_be_set_to_any_mode(self):
        """Goes through the valid modes and checks that the gpio can be set"""
        for mode in ("INPUT", "OUTPUT", "INPUT_ANALOG", "INPUT_PULLUP"):
            for pin in (1,2,3,4):
                with self.subTest(mode=mode, pin=pin):
                    self.r.gpio[pin].mode = mode

    # TODO check that anaglogue reads return something

    def test_robot_power_down(self):
        """robot.stop should set the motors to zero and the 12v off"""
        self.r.stop()
        for motor in self.r.motors:
            self.assertEqual(motor, 0)
        self.assertFalse(self.r.enable_12v)

    def test_set_12v(self):
        """Check that the 12v can be set an unset.

        We take it on faith that the interface is correct.
        """
        # Leave 12v enabled for the next test
        for state in (True, True, False, False, True):
            self.r.enable_12v = state
            self.assertEqual(self.r.enable_12v, state)

    def test_r_zone(self):
        """R.zone returns an integer, a more comprehensive test is in
        the RobotInit Class but this is a nice smoke test
        """
        self.assertTrue(type(self.r.zone) is int)


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
                R.motors[motor] = 0  # Reset the motor state for next test

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

    def test_default_logging_level(self):
        """Check that the logging level is set to logging.INFO by default"""
        R = robot.Robot()
        _logger = logging.getLogger("robot")
        self.assertEqual(_logger.getEffectiveLevel(), logging.INFO)

    def test_logging_level_adjusted(self):
        """The logging level should be able to be set by passing a kwarg into
        the robot object.
        """
        R = robot.Robot(logging_level=logging.DEBUG)
        _logger = logging.getLogger("robot")
        self.assertEqual(_logger.getEffectiveLevel(), logging.DEBUG)

    def test_robot_is_singleton(self):
        """Test that the robot can only be initialized once"""
        R1 = robot.Robot()
        with self.assertRaises(RuntimeError):
            R2 = robot.Robot()

    def test_hardware_status_printed(self):
        """The robot object should print a hardware status report upon
        initalisation, this test just checks that something like it is printed
        it should be pretty obvious if not.
        """
        with harness.captured_output() as (out, err):
            robot.Robot()

        output = out.getvalue().strip()
        print(output)
        self.assertTrue("HARDWARE REPORT" in output)
        self.assertTrue("---------------------------" in output)
        # --------------------------- is the final line of the hardware report

    # def test_r_zone_can_be_set(self):
    #     """R.zone is read from a json via a FIFO pipe, initialize this FIFO with
    #     different parameters to check r.zone can be set
    #     """
    #     USER_FIFO_PATH = mktemp(prefix="shepherd-fifo-")
    #     os.mkfifo(USER_FIFO_PATH)
    #     atexit.register(partial(os.remove, USER_FIFO_PATH))
