"""Gets frames from a camera, processes them with AprilTags and performs
postprocessing on the data to make it accessible to the user
"""
import abc
import logging
import os
import threading
import queue

from datetime import datetime
from typing import NamedTuple, Any

import cv2
import numpy as np
import picamera
import picamera.array
# seperate import of picamera.array required:
# <https://picamera.readthedocs.io/en/latest/api_array.html>
import pprint

import robot.apriltags3 as AT


class MarkerInfo(NamedTuple):
    """Marker Info which is independent of a robot"""
    code: int
    type_: str #  Rename from type to avoid name collision?
    size: float
    bounding_box_colour: tuple


class Marker():
    """A class to automatically pull the dis and bear_y out of the detection"""
    def __init__(self, info, detection):
        self.info = info
        self.detection = detection
        self.dist = detection.dist
        self.bear = detection.bear
        self.rot = detection.rot
        self.code = info.code
        self.type = info.type_

    def __repr__(self):
        """A full string representation"""
        return str(self.__dict__)

    def __str__(self):
        """A reduced set of the attributes and description text"""
        reduced_attributes = {"code": self.code,
                              "dist": self.dist,
                              "bear.y": self.bear.y,
                              "type": self.type,}
        return pprint.pformat(reduced_attributes)


class Detections(list):
    """A mutable return type for R.see"""
    def __str__(self):
        """Uses `str` instead of `repr` on list items
        The return from R.see should be humman readable"""
        return "\n".join([str(m) for m in self])



class Capture(NamedTuple):
    """Allows for passing captures around particularly to the postprocessor"""
    grey_frame: Any
    colour_frame: Any
    colour_type: Any
    time: Any


_AT_PATH = "/home/pi/apriltag"
_USB_IMAGES_PATH = "/media/RobotUSB/collect_images.txt"
_USB_LOGS_PATH = "/media/RobotUSB/log_markers.txt"

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
MARKER_ARENA, MARKER_TOKEN, MARKER_DEFAULT = "arena", "token", "default"

# NOTE Data about each marker
#     MARKER_OFFSET: Offset
#     MARKER_COUNT: Number of markers of type that exist
#     MARKER_SIZE: Real life size of marker
#         The numbers here (e.g. `0.25`) are in metres -- the 10/12 is a scaling
#         factor so that april_tags gets the size of the 10x10 black/white
#         portion (not including the white border), but so that humans can
#         measure sizes including the border.
#     MARKER_COLOUR: Bounding box colour
marker_types = {
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
    },
    MARKER_DEFAULT: {
        MARKER_OFFSET: 40,
        MARKER_COUNT: 200,
        MARKER_SIZE: 0.14 * (10.0 / 12), # This size is meaningless
        MARKER_COLOUR: WHITE
    },
}

# Creates a lookup table with all of the markers in the game
MARKER_LUT = {}
for type_, properties in marker_types.items():
    for n in range(properties[MARKER_COUNT]):
        code = properties[MARKER_OFFSET] + n
        m = MarkerInfo(code=code,
                       type_=type_,
                       size=properties[MARKER_SIZE],
                       bounding_box_colour=properties[MARKER_COLOUR])
        MARKER_LUT[code] = m

# Image post processing constants
BOUNDING_BOX_THICKNESS = 2
DEFAULT_BOUNDING_BOX_COLOUR = WHITE

# Magic number's which lets AT calculate distance different for every camera
PI_CAMERA_FOCAL_LENGTHS = {
    (640, 480): (607.6669874845361, 607.6669874845361),
    (1296, 736): (1243.0561163806915, 1243.0561163806915),
    (1296, 976): (1232.4906991188611, 1232.4906991188611),
    (1920, 1088): (3142.634753484673, 3142.634753484673),
    (1920, 1440): (1816.5165227051677, 1816.5165227051677)
}

LOGITECH_C270_FOCAL_LENGTHS = {  # fx, fy tuples
    (640, 480): (607.6669874845361, 607.6669874845361),
    (1296, 736): (1243.0561163806915, 1243.0561163806915),
    (1296, 976): (1232.4906991188611, 1232.4906991188611),
    (1920, 1088): (3142.634753484673, 3142.634753484673),
    (1920, 1440): (1816.5165227051677, 1816.5165227051677)
}


class Camera(abc.ABC):
    """Define the interface for what a camera should support"""
    params = None # (fx, fy, cx, cy) tuples

    @abc.abstractproperty
    def res(self) -> tuple:
        """Return a tuple for the current res (w, h)"""

    @res.setter
    def res(self, res: tuple) -> None:
        """This method sets the resolution of the camera it should raise an
           error if the camera failed to set the requested resolution"""

    @abc.abstractmethod
    def capture(self) -> Capture:
        """Get a frame from the camera"""

    @abc.abstractmethod
    def close(self) -> None:
        """Closes any locks that the program might have on hardware"""

    def _update_camera_params(self, focal_lengths):
        """Calculates and sets `self.params` to the correct `fx, fy, cx, cy`
        fx: focal_length_x
        cx: focal_length_x
        """
        focal_length = focal_lengths[self.res]
        center = [i/2 for i in self.res]
        self.params = (*focal_length, *center)


class RoboConPiCamera(Camera):
    """A wrapper for the PiCamera class providing the methods which are used by
    the robocon classes"""

    def __init__(self, start_res=(1296, 736), focal_lengths=None):
        self._pi_camera = picamera.PiCamera(resolution=start_res)
        self.focal_lengths = (PI_CAMERA_FOCAL_LENGTHS
                              if focal_lengths is None
                              else focal_lengths)
        self._update_camera_params(self.focal_lengths)

    @property
    def res(self):
        return self._pi_camera.resolution

    @res.setter
    def res(self, new_res: tuple):
        if new_res is not self._pi_camera.resolution:
            self._pi_camera.resolution = new_res
            actual = self._pi_camera.resolution

            assert actual == new_res, (
                f"Failed to set PiCam res, expected {new_res} but got {actual}")

            self._update_camera_params(self.focal_lengths)


    def capture(self):
        # TODO Make this return the YUV capture
        with picamera.array.PiRGBArray(self._pi_camera) as stream:
            self._pi_camera.capture(stream, format="bgr", use_video_port=True)
            capture_time = datetime.now()
            colour_frame = stream.array
            grey_frame = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)

        return Capture(grey_frame=grey_frame,
                       colour_frame=colour_frame,
                       colour_type="RGB",
                       time=capture_time)

    def close(self):
        """Prevent the picamera leaking GPU memory"""
        self._pi_camera.close()


class RoboConUSBCamera(Camera):
    """A wrapper class for the open CV methods"""
    def __init__(self,
                 start_res=(1296, 736),
                 focal_lengths=None):
        self._cv_capture = cv2.VideoCapture(0)
        self._res = start_res
        self.focal_lengths = (LOGITECH_C270_FOCAL_LENGTHS
                              if focal_lengths is None
                              else focal_lengths)
        self._update_camera_params(self.focal_lengths)

    @property
    def res(self):
        return self._res

    @res.setter
    def res(self, new_res):
        if new_res is not self._res:
            cv_property_ids = (cv2.CV_CAP_PROP_FRAME_WIDTH,
                               cv2.CV_CAP_PROP_FRAME_HEIGHT)

            for new, property_id in zip(new_res, cv_property_ids):
                self._cv_capture.set(property_id, new)
                actual = self._cv_capture.get(property_id, new)
                assert actual == new, (f"Failed to set USB res, expected {new} "
                                       f"but got {actual}")

            self._res = new_res
            self._update_camera_params(self.focal_lengths)

    def capture(self):
        """Capture from a USB camera. Not all usb cameras support native YUV
        capturing so to ensure that we have the best USB camera compatibility
        we take the performance hit and capture in RGB and covert to grey."""
        # TODO I'm sure openCV has some faster capture methods from video
        # streams like libkoki used to do with V4L.

        cam_running, colour_frame = self._cv_capture.read()
        capture_time = datetime.now()

        if not cam_running:
            raise IOError("Capture from USB camera failed")

        grey_frame = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2GRAY)

        return Capture(grey_frame=grey_frame,
                       colour_frame=colour_frame,
                       colour_type="RGB",
                       time=capture_time)

    def close(self):
        """Close the openCV capture
        OpenCV does this anyway on a call to `open` but it is good for
        consistency
        """
        self._cv_capture.release()


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
        self._bounding_box = bounding_box
        self._usb_stick = usb_stick
        self._send_to_sheep = send_to_sheep
        self._save = save

        self._stop_event = threading.Event()
        self._stop_event.clear()

        # This calls our overridden `run` method
        self.start()

    def stop(self):
        """Finnish current work then join main process"""
        self._stop_event.set()
        self.join()

    def stopped(self):
        """public alias for _stop_event"""
        return self._stop_event.is_set()

    def _draw_bounding_box(self, frame, detections):
        """Takes a frame and a list of markers drawing bounding boxes
        """
        polygon_is_closed = True
        for detection in detections:
            marker_info_colour = MARKER_LUT[detection.id].bounding_box_colour
            colour = (marker_info_colour
                      if marker_info_colour is not None
                      else DEFAULT_BOUNDING_BOX_COLOUR)

            # need to have this EXACT integer_corners syntax due to opencv bug
            # https://stackoverflow.com/questions/17241830/
            integer_corners = detection.corners.astype(np.int32)
            cv2.polylines(frame,
                          [integer_corners],
                          polygon_is_closed,
                          colour,
                          thickness=self._bounding_box_thickness)

        return frame

    @staticmethod
    def _write_to_usb(capture, detections):
        """If certain files exist on the RobotUSB writes data"""
        capture_time = str(int(capture.time))

        if os.path.exists(_USB_IMAGES_PATH):
            filename = "/media/RobotUSB/" + capture_time + ".jpg"
            cv2.imwrite(filename, capture.colour_frame)

        if os.path.exists(_USB_LOGS_PATH):
            with open(_USB_LOGS_PATH, 'a') as usb_logs:
                log_message = f"---{capture_time}---\n{detections}\n\n"
                usb_logs.write(log_message)

    def run(self):
        """This method runs in a separate process, and awaits for there to be
        data in the queue, we need to wait for there to be frames to prcess. It
        times out once a second so that we can check weather we should have
        stopped processing.
        """
        while not self._stop_event.is_set():
            try:
                # TODO do we need pass colour infomation?
                # pylint: disable=unused-variable
                (capture, detections) = (
                    self._owner.frames_to_postprocess.get(timeout=1))
            except queue.Empty:
                pass
            else:
                frame = capture.colour_frame
                if self._bounding_box:
                    frame = self._draw_bounding_box(frame, detections)
                if self._save:
                    cv2.imwrite("/tmp/colimage.jpg", frame)
                if self._usb_stick:
                    self._write_to_usb(capture, detections)
                if self._send_to_sheep:
                    pass


class Vision():
    """Class for setting camera hardware, capturing, assigning attributes
        calling the post processor"""

    def __init__(self,
                 zone,
                 at_path=_AT_PATH,
                 max_queue_size=4,
                 camera=None):

        self.zone = zone

        self.marker_info_lut = MARKER_LUT
        self.marker_size_lut = {}
        for code, properties in MARKER_LUT.items():
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

        self.camera = camera

        self.frames_to_postprocess = queue.Queue(max_queue_size)
        self.post_processor = PostProcessor(self)

    def stop(self):
        """Cleanup to prevent leaking hardware resource"""
        self.post_processor.stop()
        self.camera.close()

    @staticmethod
    def _generate_marker_properties(tags):
        """Adds `MarkerInfo` to detections"""
        detections = Detections()

        for tag in tags:
            if tag.id not in MARKER_LUT:
                logging.warning("Detected tag with id %i but not found in lut",
                                tag.id)
                continue

            info = MARKER_LUT[int(tag.id)]
            detections.append(Marker(info, tag))

        return detections

    def _send_to_post_process(self, capture, detections):
        """Places data on the post processor queue with error handeling"""
        try:
            robot_picture = (capture, detections)
            self.frames_to_postprocess.put(robot_picture, timeout=1)
        except queue.Full:
            logging.warning("Skipping postprocessing as queue is full")

    def detect_markers(self):
        """Returns the markers the robot can see:
            - Gets a frame
            - Finds the markers
            - Appends RoboCon specific properties, e.g. token or arena
            - Sends off for post processing
        """
        capture = self.camera.capture()

        detections = self.at_detector.detect(capture.grey_frame,
                                             estimate_tag_pose=True,
                                             camera_params=self.camera.params,
                                             tag_size_lut=self.marker_size_lut)

        self._send_to_post_process(capture, detections)

        markers = self._generate_marker_properties(detections)

        return markers
