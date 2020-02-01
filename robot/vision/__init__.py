"""
A vision module for detecting April tags using the robocon kit
TODO: This should be multiple files when everything is implemented
"""
import abc #Abstract-base-class
import functools
import cv2
import os
import multiprocessing
import picamera
import picamera.array
import logger
import vision.apriltags3 as AT
from datetime import datetime
# required see <https://picamera.readthedocs.io/en/latest/api_array.html>


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
    def __init__(self, start_res):
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
    """A wrapper for the PiCamera class providing the methods which are used by
    the robocon classes"""
    def __init__(self, start_res=(1296, 736)):
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

        self.start()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def run(self):
        """This method runs in a separate process, and awaits for there to be
        data in the queue, we need to wait for there to be frames to prcess. It
        times out once a second so that we can check weather we should have
        stopped processing."""
        while not self.terminated:
            try:
                frame, markers = self.owner.frames_to_postprocess.get(timeout=1)
            except Queue.Empty:
                pass
            else:
                if self.bounding_box_enable:
                    #Should this include the markers not found in the LUT?
                    for m in markers:
                        try:
                            bounding_box_colour = m.info.bounding_box_colour
                        except AttributeError:
                            bounding_box_colour = WHITE

                        #get the image cords of the diganoally oposite vertecies
                        vertex1 = (int(m.vertices[0].image.x),
                                    int(m.vertices[0].image.y))
                        vertex3 = (int(m.vertices[2].image.x),
                                    int(m.vertices[2].image.y))
                        cv2.rectangle(frame,
                                        vertex1,
                                        vertex3,
                                        bounding_box_colour,
                                        self.bounding_box_thickness)
                #End if bounding_box_enable

                cv2.imwrite("/tmp/colimage.jpg", frame)



class Vision(object):
    """A class to provide and interface and utilities for vision"""
    def __init__(self, mode, arena, zone, at_path="/home/pi/apriltag", max_queue_size=2):
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

        self.frames_to_postprocess = multiprocessing.Queue(max_queue_size)
        self.post_processor = PostProcessor(self)


    def __del__(self):
        self.post_processor.shutdown()


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