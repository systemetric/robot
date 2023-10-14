"""Gets frames from a camera, processes them with AprilTags and performs
postprocessing on the data to make it accessible to the user
"""
import abc
import functools
import logging
import os
import threading
import queue
import subprocess as sp
import re
import time

from datetime import datetime
from typing import NamedTuple, Any

from robot.sheepdog_trials.markers import MARKER
from robot.sheepdog_trials.teams import TEAM
from .sheepdog_trials import BASE_MARKER as MarkerInfo

import cv2
import numpy as np
import picamera

try:
    import picamera.array
    # seperate import of picamera.array required:
    # <https://picamera.readthedocs.io/en/latest/api_array.html>
except ImportError:
    pass  # Mock RPI doesn't have picamera.array as a separate import

import robot.apriltags3 as AT

# TODO put all of the paths together
IMAGE_TO_SHEPHERD_PATH = "/home/pi/shepherd/shepherd/static/image.jpg"


class Marker():
    """A class to automatically pull the dis and bear_y out of the detection"""

    def __init__(self, info: MarkerInfo, detection: AT.Detection):
        self.info = info
        self.detection = detection
        self.dist = detection.dist
        self.bearing = detection.bearing
        self.rotation = detection.rotation

    def __repr__(self):
        """A full string representation"""
        return str(self.__dict__)

    def __str__(self):
        """A reduced set of the attributes and description text"""
        return (f"{self.info.type} Marker {self.info.id}: {self.dist:.3}m @"
                f"{self.bearing.y:.3} degrees\n"
                "{\n"
                f"  info.type = {self.info.id}\n"
                f"  info.id = {self.info.id}\n"
                f"  info.owning_team or info.owner = {self.info.owning_team or self.info.owner}\n"
                f"  dist = {self.dist:.3}\n"
                f"  bearing.y = {self.bearing.y:.3}\n"
                f"  bearing.x = {self.bearing.x:.3}\n"
                f"  rotation.y = {self.rotation.y:.3}\n"
                f"  rotation.x = {self.rotation.x:.3}\n"
                f"  rotation.z = {self.rotation.z:.3}\n"
                f"  info = TOO BIG TO PRINT\n"
                f"  detection = TOO BIG TO PRINT\n"
                "}\n")


class Detections(list):
    """A mutable return type for R.see"""

    def __str__(self):
        """Uses `str` instead of `repr` on list items
        The return from R.see should be humman readable"""
        return "[" + "\n".join([str(m) for m in self]) + "]"


class Capture:
    """Allows for passing captures around particularly to the postprocessor"""

    def __init__(self):
        self.colour = None
        self.grey = None
        self.timestamp = None


_AT_PATH = "/usr/local"
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
# NOTE Data about each marker
#     MARKER_OFFSET: Offset
#     MARKER_COUNT: Number of markers of type that exist
#     MARKER_SIZE: Real life size of marker
#         The numbers here (e.g. `0.25`) are in metres
#         the we are using come as a 10x10 square the outer ring of
#         which is white. The size here includes this white boarder.
#     MARKER_COLOUR: Bounding box colour

# Image post processing constants
BOUNDING_BOX_THICKNESS = 2
DEFAULT_BOUNDING_BOX_COLOUR = WHITE

# Magic number's which lets AT calculate distance different for every camera
PI_CAMERA_FOCAL_LENGTHS = {
    (640, 480): (401.5, 401.5),
    (1280, 720): (821, 821),
    (1296, 736): (821, 821),
    (1296, 976): (821, 821),
    (1920, 1088): (2076, 2076),
    (1920, 1440): (1198, 1198)
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
    params = None  # (fx, fy, cx, cy) tuples

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
        center = [i / 2 for i in self.res]
        self.params = (*focal_length, *center)


def pi_cam_capture(cam, capture, lock, img_queue):
    while True:
        with picamera.array.PiRGBArray(cam) as stream:
            cam.capture(stream, format="bgr", use_video_port=True)
            img = stream.array
        lock.acquire()
        capture.colour = img
        capture.grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        capture.timestamp = time.perf_counter()
        lock.release()
        img_queue.append(img)
        time.sleep(0.001)


def prepare_for_stream(img_queue):
    while True:
        if img_queue:
            img = img_queue.pop()
            cv2.imwrite("/tmp/in_progress_capture.jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])
            os.rename("/tmp/in_progress_capture.jpg", "/tmp/current.jpg")
        else:
            time.sleep(0.02)


class RoboConPiCamera(Camera):
    """A wrapper for the PiCamera class providing the methods which are used by
    the robocon classes"""

    def __init__(self, start_res=(1280, 720), focal_lengths=None):
        self._pi_camera = picamera.PiCamera(resolution=start_res)
        self.focal_lengths = (PI_CAMERA_FOCAL_LENGTHS
                              if focal_lengths is None
                              else focal_lengths)
        self._update_camera_params(self.focal_lengths)
        self.latest_capture = Capture()
        self.lock = threading.Lock()
        self.queue = []
        self.thread = threading.Thread(target=functools.partial(pi_cam_capture, self._pi_camera, self.latest_capture,
                                                                self.lock, self.queue))
        self.thread.start()

        self.stream_thread = threading.Thread(target=functools.partial(prepare_for_stream, self.queue))
        self.stream_thread.start()

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
        capture = Capture()
        while True:
            self.lock.acquire()
            capture.colour = self.latest_capture.colour
            capture.grey = self.latest_capture.grey
            capture.timestamp = self.latest_capture.timestamp
            self.lock.release()
            if capture.colour is not None:
                break

        # print(time.perf_counter() - capture.timestamp, "cam")

        return capture

    def close(self):
        """Prevent the picamera leaking GPU memory"""
        self._pi_camera.close()


def usb_cam_capture(cam, capture, lock, img_queue):
    while True:
        cam_running, img = cam.read()

        if not cam_running:
            raise IOError("Capture from USB camera failed")

        lock.acquire()
        capture.colour = img
        capture.grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        capture.timestamp = time.perf_counter()
        lock.release()
        img_queue.append(img)
        time.sleep(0.001)


class RoboConUSBCamera(Camera):
    """A wrapper class for the open CV methods"""

    def __init__(self,
                 start_res=(1296, 736),
                 focal_lengths=None):
        self._source = self.find_usb_cam()
        if self._source is None:
            raise Exception("No USB camera detected, please make sure it's plugged in")
        self._cv_capture = cv2.VideoCapture(self._source)
        self._res = start_res
        self.focal_lengths = (LOGITECH_C270_FOCAL_LENGTHS
                              if focal_lengths is None
                              else focal_lengths)
        self._update_camera_params(self.focal_lengths)

        self.latest_capture = Capture()
        self.lock = threading.Lock()

        self.queue = []
        self.thread = threading.Thread(target=functools.partial(usb_cam_capture,
                                                                self._cv_capture, self.latest_capture,
                                                                self.lock, self.queue))
        self.thread.start()

        self.stream_thread = threading.Thread(target=functools.partial(prepare_for_stream, self.queue))
        self.stream_thread.start()

    def find_usb_cam(self):
        options = filter(lambda file: file.startswith("video"),
                         os.listdir("/dev/"))
        for option in options:
            try:
                output = sp.check_output(["v4l2-ctl", "-d", f"/dev/{option}", "-D"]).decode()
            except sp.CalledProcessError:
                continue

            bus_info = re.search(r"Bus info *: (.*)", output)

            if bus_info is None or "usb" not in bus_info.group(1):
                # If this isn't a USB camera...
                continue

            video_capture_capabilities = re.search(
                r"Device Caps *: .*(\n\t\t(.*))*\n\t\tVideo Capture\n", output, re.MULTILINE
            )

            if video_capture_capabilities is not None:
                # If we have Video Capture listed under capabilities
                return f"/dev/{option}"

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

        capture = Capture()
        while True:
            self.lock.acquire()
            capture.colour = self.latest_capture.colour
            capture.grey = self.latest_capture.grey
            capture.timestamp = self.latest_capture.timestamp
            self.lock.release()
            if capture.colour is not None:
                break

        # print(time.perf_counter() - capture.timestamp, "cam")

        return capture

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
                 zone,
                 bounding_box_thickness=5,
                 bounding_box=True,
                 usb_stick=False,
                 send_to_sheep=False,
                 save=True):

        super(PostProcessor, self).__init__()

        self._owner = owner
        self.zone = zone
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
        """Finish current work then join main process"""
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
            marker_info = MARKER.by_id(detection.id, self.zone)
            marker_info_colour = marker_info.bounding_box_color
            marker_code = detection.id
            colour = (marker_info_colour
                      if marker_info_colour is not None
                      else DEFAULT_BOUNDING_BOX_COLOUR)

            # need to have this EXACT integer_corners syntax due to opencv bug
            # https://stackoverflow.com/questions/17241830/
            integer_corners = detection.corners.astype(np.int32)

            if (marker_info.owning_team == self.zone):
                cv2.polylines(frame,
                              [integer_corners],
                              polygon_is_closed,
                              colour,
                              thickness=self._bounding_box_thickness * 3)
            else:
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
                (capture, detections) = self._owner.frames_to_postprocess.get(timeout=1)
            except queue.Empty:
                pass
            else:
                frame = capture
                if self._bounding_box:
                    frame = self._draw_bounding_box(frame, detections)
                if self._save:
                    cv2.imwrite(IMAGE_TO_SHEPHERD_PATH, frame)
                if self._usb_stick:
                    self._write_to_usb(capture, detections)
                if self._send_to_sheep:
                    pass


class ATDetections:
    def __init__(self):
        self.output = None
        self.frame = None
        self.timestamp = None


def detect_markers(camera, at_detector, detections, lock):
    while True:
        capture = camera.capture()
        # print(time.perf_counter() - capture.timestamp, "marker")
        s = time.perf_counter()
        output = at_detector.detect(capture.grey, estimate_tag_pose=True,
                                    camera_params=camera.params)
        # print(time.perf_counter() - capture.timestamp, "marker")
        # print(time.perf_counter() - s)
        lock.acquire()
        detections.output = output
        detections.frame = capture.colour
        detections.timestamp = capture.timestamp
        lock.release()


class Vision():
    """Class for setting camera hardware, capturing, assigning attributes
        calling the post processor"""

    def __init__(self,
                 zone,
                 at_path=_AT_PATH,
                 max_queue_size=4,
                 camera=None):

        self.zone = zone

        at_lib_path = (
            "{}/lib".format(at_path),
            "{}/lib64".format(at_path)
        )

        self.at_detector = AT.Detector(searchpath=at_lib_path,
                                       families="tag36h11",
                                       nthreads=4,
                                       quad_decimate=2.0,
                                       quad_sigma=0.0,
                                       refine_edges=1,
                                       decode_sharpening=0.25,
                                       debug=0)

        self.camera = camera

        self.detections = ATDetections()
        self.lock = threading.Lock()
        self.at_thread = threading.Thread(target=functools.partial(detect_markers, self.camera,
                                                                   self.at_detector, self.detections, self.lock))
        self.at_thread.start()

        self.frames_to_postprocess = queue.Queue(max_queue_size)
        self.post_processor = PostProcessor(self, zone=self.zone)

    def stop(self):
        """Cleanup to prevent leaking hardware resource"""
        self.post_processor.stop()
        self.camera.close()

    def _generate_marker_properties(self, tags):
        """Adds `MarkerInfo` to detections"""
        detections = Detections()

        for tag in tags:
            info = MARKER.by_id(int(tag.id), self.zone)
            detections.append(Marker(info, tag))

        return detections

    def _send_to_post_process(self, capture, detections):
        """Places data on the post processor queue with error handling"""
        try:
            robot_picture = (capture, detections)
            self.frames_to_postprocess.put(robot_picture, timeout=1)
        except queue.Full:
            logging.warning("Skipping postprocessing as queue is full")

    def detect_markers(self, return_frame=False):
        """Returns the markers the robot can see:
            - Gets a frame
            - Finds the markers
            - Appends RoboCon specific properties, e.g. token or arena
            - Sends off for post-processing
        """
        start_timestamp = time.perf_counter()
        self.lock.acquire()
        detections = self.detections.output
        capture = self.detections.frame
        timestamp = self.detections.timestamp
        self.lock.release()
        while timestamp < start_timestamp:
            self.lock.acquire()
            detections = self.detections.output
            capture = self.detections.frame
            timestamp = self.detections.timestamp
            self.lock.release()

        self._send_to_post_process(capture, detections)

        markers = self._generate_marker_properties(detections)

        # print(time.perf_counter() - timestamp, "final")
        if return_frame:
            return markers, capture
        else:
            return markers
