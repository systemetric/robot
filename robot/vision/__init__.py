"""
A vision module for detecting April tags using the robocon kit
"""
import abc #Abstract-base-class
import functools
import cv2
import os
import multiprocessing
import picamera
import picamera.array
# required see <https://picamera.readthedocs.io/en/latest/api_array.html>
import logger
import vision.apriltags3 as AT
from datetime import datetime


# Camera details [fx, fy, cx, cy]
camera_params = [336.7755634193813, 336.02729840829176,
                 333.3575643300718, 212.77376312080065]

PI_CAMERA_FOCAL_LENGTHS = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

LOGITECH_C270_FOCAL_LENGTHS = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

MAKER_ARENA, MAKER_TOKEN = "arena", "token"

maker_sizes = {
    MAKER_ARENA: 0.25,
    MAKER_TOKEN: 0.1,
}

marker_size_lut = dict([(i, marker_sizes["MARKER_TOKEN"]) for i in range(100)])

MarkerInfo = namedtuple("MarkerInfo", "code marker_type token_type offset size")
ImageCoord = namedtuple("ImageCoord", "x y")

# Define a tuple for passing captured frames around, colour frames for speed
# are stored in whatever encoding RGB, BGR, YUV which they were captured in
# it is upto the postprocessor to deal with this.
Capture = namedtuple("Capture", "grey_frame", "colour_frame", "colour_type", "time")

class Camera(abc.ABC):
    """An abstract class which defines the methods the cameras must support"""

    @abc.abstractmethod
    def set_res(self, res):
        """This method sets the resolution of the camera it should raise an
           error if the camera failed to set the requested resolution"""

    @abc.abstractmethod
    def capture(self):
        """This method should return a YUV numpy array with data from the sensor
        """


class RoboConPiCamera(Camera):
    """A wrapper for the PiCamera class providing the methods which are used by
    the robocon classes"""
    def __init__(self, start_res=(1296, 736)):
        self._camera = picamera.PiCamera(resolution=start_res)

    def set_res(self, res):
        if res == self._camera.resolution:
            # Resolution already the requested one
            return

        try:
            self._camera.resolution = res
        except Exception as e:
            raise ValueError(f"Setting camera resolution failed with {type(e)}")

        actual = self._camera.resolution
        if res != actual:
            raise ValueError(f"Unsupported image resolution {res} (got: {actual})")

    def capture(self):
        # TODO Make this return the YUV capture
        with picamera.array.PiRGBArray(self._camera) as stream:
            self._camera.capture(stream, format="bgr", use_video_port=True)
            capture_time = datetime.time()
            colour_frame = stream.array
            grey_frame = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)

        result = Capture(grey_frame=grey_frame,
                         colour_frame=colour_frame,
                         colour_type="RGB",
                         time=capture_time)
        return result


class RoboConUSBCamera(Camera):
    """A wrapper class for the open CV methods for generic cameras"""
    def __init__(self, start_res=(1296, 736), lut=LOGITECH_C270_FOCAL_LENGTHS)
        self._camera = cv2.VideoCapture(0)
        self.res = start_res

    def set_res(self, new_res):
        if self.res != new_res:
            self._camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, new_res[0])
            self._camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, new_res[1])

        self.res = new_res

    def capture(self):
        """Capture from a USB camera. Not all usb cameras support native YUV
        capturing so to ensure that we have the best USB camera compatibility
        we take the performance hit and capture in RGB and covert to grey."""
        #TODO I'm sure openCV has some faster capture methods from video
        # streams like libkoki used to do.

        cam_running, colour_frame = self._camera.read()
        capture_time = datetime.now()

        if not cam_running:
            raise IOError("Capture from USB camera failed")

        grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        result = Capture(grey_frame=grey_frame,
                         colour_frame=colour_frame,
                         colour_type="RGB",
                         time=capture_time)
        return result

class PostProcessor(multiprocessing.Process):
    """Once AprilTags returns its marker properties then there convince outputs
    todo e.g. send the image over to sheep. To make R.see() as quick as possible
    we do this asynchronously in another process to avoid the GIL.

    Note: because AprilTags can use all 4 cores that the pi has this still isn't
    free if we are processing frames back to backs."""
    def __init__(self,
                    owner,
                    bounding_box_enable=True,
                    bounding_box_thickness=2):

        super(PostProcessor, self).__init__()

        self.owner = owner
        self.bounding_box_enable = bounding_box_enable
        self.bounding_box_thickness = bounding_box_thickness

        self.terminated = False
        self.usb_stick = False
        self.send_to_sheep = False

        # Sqwan the new process
        self.start()

    def stop(self):
        self._stop_event.set()
        self.join()

    def stopped(self):
        return self._stop_event.is_set()

    def draw_bounding_box(self, frame, makers):
        """Takes a frame and a list of markers drawing bounding boxes
        #TODO check if this is actually passed the frame object or if it is mutating it globally
        """
        for m in markers:
            try:
                bounding_box_colour = m.info.bounding_box_colour
            except AttributeError:
                bounding_box_colour = WHITE # should be a "default colour"

            # Shift and wrap the list by 1
            rotated_vetecies = m.vertecies.roll(1)

            for current_v, next_v in zip(vertices, rotated_vetecies):
                cv2.rectangle(frame,
                                current_v,
                                next_v,
                                bounding_box_colour,
                                self.bounding_box_thickness)

    def run(self):
        """This method runs in a separate process, and awaits for there to be
        data in the queue, we need to wait for there to be frames to prcess. It
        times out once a second so that we can check weather we should have
        stopped processing."""
        while not self.terminated:
            try:
                frame, colour_type, markers = self.owner.frames_to_postprocess.get(timeout=1)
            except Queue.Empty:
                pass
            else:
                if self.bounding_box_enable:
                    frame = self.draw_bounding_box(frame, markers)
                if save:
                    cv2.imwrite("/tmp/colimage.jpg", frame)
                if self.usb_stick:
                    pass
                if self.send_to_sheep:
                    pass

class Vision(object):
    """A class to provide and interface and utilities for vision"""
    def __init__(self, mode, arena, zone, at_path="/home/pi/apriltag", max_queue_size=2):
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

        self.frames_to_postprocess = multiprocessing.Queue(max_queue_size)
        self.post_processor = PostProcessor(self)


    def __del__(self):
        self.post_processor.stop()


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
        capture = self.camera.capture()

        detections = self.at_detector.detect(capture.grey_frame,
                                        estimate_tag_pose=True,
                                        camera_params=camera_params,
                                        tag_size_lut=marker_size_lut)

        robocon_markers = self._generate_marker_properties(detections)

        frames_to_postprocess.put(capture.colour_frame, capture.colour_type, markers)

        return robocon_markers