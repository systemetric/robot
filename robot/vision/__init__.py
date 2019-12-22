"""
A vision module for detecting April tags using the robocon kit
TODO: This should be multiple files when everything is implemented
"""
import abc #Abstract-base-class
import apriltags3_py.__init__ as AT
import cv2
import picamera
import picamera.array
# required see <https://picamera.readthedocs.io/en/latest/api_array.html>

res = (1296, 736)


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
        pass

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
        print('Initalizing vision')
        at_detector = AT.Detector(searchpath=[
                                        'apriltags3-py/apriltags/lib',
                                        'apriltags3-py/apriltags/lib64'],
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        self.camera = RoboconPiCamera()

    @staticmethod
    def _width_from_code(lut, code):
        if code not in lut:
            # TODO ignore these
            print(f"WARNING: CODE {code} NOT IN LUT {lut}")
            return 0.1
        else:
            return

    def see():
        frame = self.camera.capture()

