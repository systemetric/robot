"""
A vision module for detecting April tags using the robocon kit
"""
# TODO pylint
import abc  # Abstract-base-class
import functools
import cv2
import os
import threading
import picamera
import picamera.array
# required see <https://picamera.readthedocs.io/en/latest/api_array.html>
import logging
import robot.apriltags3 as AT
# TODO create a wraper or a PR for changes and move to dt-apriltags3
from datetime import datetime
from collections import namedtuple
import queue
import numpy as np
import pprint
import inspect

_AT_PATH = "/home/pi/apriltag"

PI_CAMERA_FOCAL_LENGTHS = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (690, 690),
    (640, 480): (500, 500),
}

LOGITECH_C270_FOCAL_LENGTHS = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}


# Colours are in the format BGR
PURPLE = (255, 0, 215)  # Purple
ORANGE = (0, 128, 255)  # Orange
YELLOW = (0, 255, 255)  # Yellow
GREEN = (0, 255, 0)  # Green
RED = (0, 0, 255)  # Red
BLUE = (255, 0, 0)  # Blue
WHITE = (255, 255, 255)  # White


# MARKER_: Marker Data Types
# MARKER_TYPE_: Marker Types
MARKER_TYPE, MARKER_OFFSET, MARKER_COUNT, MARKER_SIZE, MARKER_COLOUR = (
    'type', 'offset', 'count', 'size', 'colour')
MARKER_ARENA, MARKER_TOKEN = "arena", "token"


# NOTE Data about each marker
#     MARKER_OFFSET: Offset
#     MARKER_COUNT: Number of markers of type that exist
#     MARKER_SIZE: Real life size of marker
#         The numbers here (e.g. `0.25`) are in metres -- the 10/12 is a scaling
#         factor so that april_tags gets the size of the 10x10 black/white
#         portion (not including the white border), but so that humans can
#         measure sizes including the border.
#     MARKER_COLOUR: Bounding box colour

marker_data = {
    MARKER_ARENA: {
        MARKER_OFFSET: 0,
        MARKER_COUNT: 32,
        MARKER_SIZE: 0.14 * (10.0 / 12),
        MARKER_COLOUR: RED
    },
    MARKER_TOKEN: {
        MARKER_OFFSET: 32,
        MARKER_COUNT: 8,
        MARKER_SIZE: 0.14 * (10.0 / 12),
        MARKER_COLOUR: YELLOW
    }
}


MarkerInfo = namedtuple("MarkerInfo",
                        "code marker_type offset size bounding_box_colour")
ImageCoord = namedtuple("ImageCoord", "x y")

marker_lut = {}
for maker_type, properties in marker_data.items():
    for n in range(properties[MARKER_COUNT]):
        code = properties[MARKER_OFFSET] + n
        m = MarkerInfo(code=code,
                        marker_type=maker_type,
                        offset=n,
                        size=properties[MARKER_SIZE],
                        bounding_box_colour=properties[MARKER_COLOUR])
        marker_lut[code] = m


# Image post processing constants
BOUNDING_BOX_THICKNESS = 2
DEFAULT_BOUNDING_BOX_COLOUR = WHITE
# Define a tuple for passing captured frames around, colour frames for speed
# are stored in whatever encoding RGB, BGR, YUV which they were captured in
# it is upto the postprocessor to deal with this.
Capture = namedtuple("Capture", "grey_frame colour_frame colour_type time")


class Marker(object):
    """A class to automatically pull the dis and bear_y out of the detection"""

    def __init__(self, info, detection):
        self.info = info
        self.detection = detection
        self.dist = detection.dist
        self.bear = detection.bear
        self.rot = detection.rot

    def __str__(self):
        """A reduced set of the attributes and discription text"""
        title_text = "The contents of the %s object" % self.__class__.__name__
        reduced_attributes = self.__dict__.copy()
        del reduced_attributes["detection"]
        readable_contents = pprint.pformat(reduced_attributes)
        return "{}\n{}".format(title_text, readable_contents)


class Camera(object):
    """Define the interface for what a camera should support"""

    def __init__(self, focal_lengths):
        self.params = None
        self.focal_lengths = focal_lengths
        self._update_camera_params()

    @abc.abstractproperty
    def res(self):
        """Return a tuple for the current res (w, h)"""

    @abc.abstractproperty
    @res.setter
    def res(self, res):
        """This method sets the resolution of the camera it should raise an
           error if the camera failed to set the requested resolution"""

    @abc.abstractmethod
    def capture(self):
        """This method should return a Capture named tuple
        """

    def _update_camera_params(self):
        """Returns a `fx, fy, cx, cy`"""
        focal_length = self.focal_lengths[self.res]
        center = [i/2 for i in self.res]
        self.params = (*focal_length, *center)


class RoboConPiCamera(Camera):
    """A wrapper for the PiCamera class providing the methods which are used by
    the robocon classes"""

    def __init__(self,
                 start_res=(1296, 736),
                 focal_lengths=PI_CAMERA_FOCAL_LENGTHS):
        self._pi_camera = picamera.PiCamera(resolution=start_res)
        super().__init__(focal_lengths)

    @property
    def res(self):
        return self._pi_camera.resolution

    @res.setter
    def res(self, res):
        assert res in self.focal_lengths

        if res == self._pi_camera.resolution:
            return True

        try:
            self._pi_camera.resolution = res
        except Exception as e:
            raise ValueError(
                "Setting camera resolution failed with {}".format(type(e)))

        actual = self._pi_camera.resolution
        if res != actual:
            raise ValueError(
                "Unsupported image resolution {} (got: {})".format(res, actual))

        self._update_camera_params()

    def capture(self):
        # TODO Make this return the YUV capture
        with picamera.array.PiRGBArray(self._pi_camera) as stream:
            self._pi_camera.capture(stream, format="bgr", use_video_port=True)
            capture_time = datetime.now()
            colour_frame = stream.array
            grey_frame = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)

        result = Capture(grey_frame=grey_frame,
                         colour_frame=colour_frame,
                         colour_type="RGB",
                         time=capture_time)
        return result


# TODO actually test this works
class RoboConUSBCamera(Camera):
    """A wrapper class for the open CV methods for generic cameras"""

    def __init__(self,
                 start_res=(1296, 736),
                 focal_lengths=LOGITECH_C270_FOCAL_LENGTHS):
        self._cv_camera = cv2.VideoCapture(0)
        self._res = start_res
        super().__init__(focal_lengths)

    @property
    def res(self):
        return self._res

    @res.setter
    def res(self, new_res):
        self._cv_camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, new_res[0])
        self._cv_camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, new_res[1])

        self._res = new_res
        self._update_camera_params()

    def capture(self):
        """Capture from a USB camera. Not all usb cameras support native YUV
        capturing so to ensure that we have the best USB camera compatibility
        we take the performance hit and capture in RGB and covert to grey."""
        # TODO I'm sure openCV has some faster capture methods from video
        # streams like libkoki used to do with V4L.

        cam_running, colour_frame = self._cv_camera.read()
        capture_time = datetime.now()

        if not cam_running:
            raise IOError("Capture from USB camera failed")

        grey_frame = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2GRAY)

        result = Capture(grey_frame=grey_frame,
                         colour_frame=colour_frame,
                         colour_type="RGB",
                         time=capture_time)
        return result


class PostProcessor(threading.Thread):
    """Once AprilTags returns its marker properties then there convince outputs
    todo e.g. send the image over to sheep. To make R.see() as quick as possible
    we do this asynchronously in another process to avoid the GIL.

    Note: because AprilTags can use all 4 cores that the pi has this still isn't
    free if we are processing frames back to backs.
    """

    def __init__(self,
                 owner,
                 bounding_box_thickness=2,
                 bounding_box=True,
                 usb_stick=False,
                 send_to_sheep=False,
                 save=True):

        super(PostProcessor, self).__init__()

        self._owner = owner
        self._bounding_box_thickness = bounding_box_thickness

        self.signals = {
            "bounding_box": threading.Event(),
            "usb_stick": threading.Event(),
            "send_to_sheep": threading.Event(),
            "save": threading.Event(),
        }

        for signal_name, signal in self.signals.items():
            if locals()[signal_name] is True:
                signal.set()
            else:
                signal.clear()

        self._stop_event = threading.Event()
        self._stop_event.clear()

        # Spwan the new thread
        self.start()

    def stop(self):
        self._stop_event.set()
        self.join()

    def stopped(self):
        return self._stop_event.is_set()

    def _draw_bounding_box(self, frame, detections):
        """Takes a frame and a list of markers drawing bounding boxes
        """
        polygon_is_closed = True
        for detection in detections:
            try:
                colour = marker_lut[detection.id].bounding_box_colour
            except AttributeError:
                # TODO check if this is the correct error to catch?
                colour = DEFAULT_BOUNDING_BOX_COLOUR

            # need to have this EXACT integer_corners syntax due to opencv bug
            # https://stackoverflow.com/questions/17241830/opencv-polylines-function-in-python-throws-exception
            integer_corners = detection.corners.astype(np.int32)
            cv2.polylines(frame,
                          [integer_corners],
                          polygon_is_closed,
                          colour,
                          thickness=self._bounding_box_thickness)

        return frame

    def run(self):
        """This method runs in a separate process, and awaits for there to be
        data in the queue, we need to wait for there to be frames to prcess. It
        times out once a second so that we can check weather we should have
        stopped processing.
        # TODO do we need pass colour infomation?
        """
        while not self._stop_event.is_set():
            try:
                (frame, colour_type, detections) = (
                    self._owner.frames_to_postprocess.get(timeout=1))
                pass
            except queue.Empty:
                pass
            else:
                bounding_box = self.signals["bounding_box"].is_set()
                save = self.signals["save"].is_set()
                usb_stick = self.signals["usb_stick"].is_set()
                send_to_sheep = self.signals["send_to_sheep"].is_set()

                if bounding_box:
                    frame = self._draw_bounding_box(frame, detections)
                if save:
                    cv2.imwrite("/tmp/colimage.jpg", frame)
                if usb_stick:
                    pass
                if send_to_sheep:
                    pass


class Vision(object):
    """Class for setting camera hardware, capturing, assigning attributes
        calling the post processor"""

    def __init__(self,
                 zone,
                 at_path=_AT_PATH,
                 max_queue_size=4,
                 use_usb_cam=False):

        self.zone = zone

        self.marker_info_lut = marker_lut
        self.marker_size_lut = {}
        for code, properties in marker_lut.items():
            self.marker_size_lut[code] = properties.size

        at_lib_path = (
            "{}/lib".format(at_path),
            "{}/lib64".format(at_path)
        )

        self.at_detector = AT.Detector(searchpath=at_lib_path,
                                       families="tag36h11",
                                       nthreads=4,
                                       quad_decimate=1.0,
                                       quad_sigma=0.0,
                                       refine_edges=1,
                                       decode_sharpening=0.25,
                                       debug=0)

        if use_usb_cam:
            self.camera = RoboConUSBCamera()
        else:
            self.camera = RoboConPiCamera()

        self._using_usb_cam = use_usb_cam

        self.frames_to_postprocess = queue.Queue(max_queue_size)
        self.post_processor = PostProcessor(self)

    def __del__(self):
        self.post_processor.stop()

    def _generate_marker_properties(self, tags):
        """Adds `MarkerInfo` to detections"""
        markers = []
        for tag in tags:
            if tag.id not in marker_lut:
                logging.warn("Detected tag with id %i but not found in lut",
                             tag.id)
                continue

            info = marker_lut[int(tag.id)]
            markers.append(Marker(info, tag))

        return markers

    def _send_to_post_process(self, colour_frame, colour_type, detections):
        """Places data on the post processor queue with error handeling"""
        try:
            capture = (colour_frame, colour_type, detections)
            self.frames_to_postprocess.put(capture, timeout=1)
        except queue.Full:
            logging.warn("Skipping postprocessing as queue is full")

    def detect_markers(self):
        """Returns the markers the robot can see:
            - Gets a frame
            - Finds the markers
            - Appends robocon specific properties, e.g. token or arena
            - Sends off for post processing
        """
        capture = self.camera.capture()

        detections = self.at_detector.detect(capture.grey_frame,
                                             estimate_tag_pose=True,
                                             camera_params=self.camera.params,
                                             tag_size_lut=self.marker_size_lut)

        self._send_to_post_process(capture.colour_frame,
                                   capture.colour_type,
                                   detections)

        markers = self._generate_marker_properties(detections)

        return markers
