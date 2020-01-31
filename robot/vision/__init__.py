"""
A vision module for detecting April tags using the robocon kit
TODO: This should be multiple files when everything is implemented
"""
import abc #Abstract-base-class
import functools
import cv2
import os
import picamera
import picamera.array
import logger
import vision.apriltags3 as AT
from datetime import datetime
# required see <https://picamera.readthedocs.io/en/latest/api_array.html>


res = (1296, 736)

# Camera details [fx, fy, cx, cy]
camera_params = [336.7755634193813, 336.02729840829176,
                 333.3575643300718, 212.77376312080065]

MAKER_ARENA, MAKER_TOKEN = "arena", "token"

maker_sizes = {
    MAKER_ARENA: 0.25,
    MAKER_TOKEN: 0.1,
}

marker_size_lut = dict([(i, marker_sizes["MARKER_TOKEN"]) for i in range(100)])

MarkerInfo = namedtuple("MarkerInfo", "code marker_type token_type offset size")
ImageCoord = namedtuple("ImageCoord", "x y")

class Marker():
    """A child class to automatically pull the dis and rot_y out of center.polar
    attribute"""
    def __init__(self, info, detection):
        # Aliases
        self.dist = detection.dist
        self.rot_y = detection.rot_y
        self.info = info
        self.detection = detection


class Logger():
    """We need to be able to write to both USB sticks as well as stdout this
    cleans this up"""
    def __init__(self):
        self.usb_log = os.path.exists("/media/RobotUSB/log_markers.txt")
        self.logfile = open("/media/RobotUSB/" + str(int(acq_time)) + ".txt", "w")

    def write(self, message):
        if self.usb_log:
            pass


class Camera(abc.ABC):
    """An abstract class which defines the methods the cameras must support"""
    @abc.abstractmethod
    def __init__(self):
        raise NotImplementedError

    @abc.abstractmethod
    def set_res(self, res):
        """This method sets the resolution of the camera it should raise an
           error if the camera failed to set the requested resolution"""
        raise NotImplementedError

    @abc.abstractmethod
    def capture(self):
        """This method should return a YUV numpy array with data from the sensor
        """
        raise NotImplementedError


class RoboconPiCamera(Camera):
    """A wrapper for the PiCamera class providing RoboCon methods"""
    def __init__(self, start_res=res):
        self.camera = picamera.PiCamera(resolution=start_res)

    def set_res(self, res):
        if res == self.camera.resolution:
            # Resolution already the requested one
            return

        try:
            self.camera.resolution = res
        except Exception as e:
            raise ValueError(f"Setting camera resolution failed with {type(e)}")

        actual = self.camera.resolution
        if res != actual:
            raise ValueError(f"Unsupported image resolution {res} (got: {
                             actual})")

    def capture(self):
        # TODO Make this return the YUV capture
        with picamera.array.PiRGBArray(self.camera) as stream:
            self.camera.capture(stream, format="bgr", use_video_port=True)
            image = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)

        return image


class Vision(object):
    """A class to provide and interface and utilities for vision"""
    def __init__(self, mode, arena, zone, at_path="/home/pi/apriltag"):
        #NO-COMMIT should the at_path be a default at this level not at the top
        # level of the robot
        self.mode = mode
        self.arena = arena
        self.zone = zone

        self.at_detector = AT.Detector(searchpath=[
                                        f'{at_path}/lib',
                                        f'{at_path}/lib64'
                                    ],
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.camera = RoboconPiCamera()


    def _generate_marker_properties(self, tags):
        """A function to return the marker objects properties in polar form"""
        markers = []
        for tag in tags:
            if tag.id not in marker_luts[self.mode][self.arena][self.zone]:
                # TODO should really be a call to the python logger
                print("WARNING: tag not in marker lut")
                continue

            info = marker_luts[mode][arena][zone][int(tag.id)]
            markers.append(Maker(info, tag))

        return markers

    def see(self, save):
        """Returns the markers the robot can see:
            - Gets a frame
            - Finds the markers
            - Posts the result to shepherd
        """
        frame = self.camera.capture()

        detections = self.at_detector.detect(frame,
                                        estimate_tag_pose=True,
                                        camera_params=camera_params,
                                        tag_size_lut=marker_size_lut)

        robocon_markers = self._generate_marker_properties(detections)

        if save:
            cv2.imwrite("tag.jpg", frame)

        return robocon_markers