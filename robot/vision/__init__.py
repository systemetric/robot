"""
A vision module for detecting April tags using the robocon kit
TODO: This should be multiple files when everything is implemented
"""
import abc #Abstract-base-class
import functools
import cv2
import picamera
import picamera.array
import vision.apriltags3 as AT
# required see <https://picamera.readthedocs.io/en/latest/api_array.html>


APRIL_PATH = "/home/pi/apriltag"
res = (1296, 736)

# Camera details [fx, fy, cx, cy]
camera_params = [336.7755634193813, 336.02729840829176, 333.3575643300718, 212.77376312080065]

MAKER_ARENA, MAKER_TOKEN = "arena", "token"

maker_sizes = {
    MAKER_ARENA: 0.25,
    MAKER_TOKEN: 0.1,
}

marker_size_lut = dict([(i, 0.1) for i in range(100)])


class Timer(object):
    """An object for timing using the with statement
       TODO: replace with timeit
    """
    def __enter__(self):
        self.start = time.time()

    def __exit__(self, t, v, tb):
        self.time = time.time() - self.start
        return False


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
        """This method should return a YUV numpy array with data from the sensor"""
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
            raise ValueError("Setting camera resolution failed with {}".format(type(e)))

        actual = self.camera.resolution
        if res != actual:
            raise ValueError("Unsupported image resolution {0} (got: {1})".format(res, actual))

    def capture(self):
        # TODO Make this return the YUV capture
        with picamera.array.PiRGBArray(self.camera) as stream:
            self.camera.capture(stream, format="bgr", use_video_port=True)
            image = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
        
        return image


class Vision(object):
    def __init__(self):
        print("Initalizing vision")
        print("Initalizing AT detector")
        self.at_detector = AT.Detector(searchpath=[
                                        f'{APRIL_PATH}/lib',
                                        f'{APRIL_PATH}/lib64'
                                    ],
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        print("Initalizing Pi Camera")
        
        self.camera = RoboconPiCamera()


    def see(self):
        """Returns the markers the robot can see:
            - Gets a frame
            - Finds the tags
            - Posts the result to shepherd
        """
        frame = self.camera.capture()

        tags = self.at_detector.detect(frame, True, camera_params, marker_size_lut)

        robocon_tags = []
        for tag in tags:
            robocon_tags.append(tag)

        save = True

        if save:
            cv2.imwrite("tag.jpg", frame)

        return robocon_tags