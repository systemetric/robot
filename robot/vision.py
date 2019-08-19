"""TODO
    When implementing USB camera preview saving bounding box/threading issues and
    deadlocks will likely be encountered. 
"""

import functools
import os
import threading
import time
from collections import namedtuple

import numpy as np
import cv2
import picamera
import picamera.array  # Required, see <https://picamera.readthedocs.io/en/latest/api_array.html>
import pykoki
# noinspection PyUnresolvedReferences
from pykoki import CameraParams, Point2Df, Point2Di


picamera_focal_lengths = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

#For now we are assuming that the USB camera is exactly like the Pi Cam
usbcamera_focal_lengths = {
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

# Source: <https://elinux.org/Rpi_Camera_Module#Technical_Parameters_.28v.2_board.29>
FOCAL_LENGTH_MM = 4.0
SENSOR_WIDTH_MM = 3.674
SENSOR_HEIGHT_MM = 2.760

# Estimate focal lengths for resolutions we don't have a focal length for.
# These aren't particularly accurate; the estimate for 1920x1080 caused a
# length of 50 cm to be reported as 60 cm.
# We should proablly document to only use the resolutions that we actually have the focal lengths for
# <http://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/?answer=17180#post-id-17180>
for res in picamera_focal_lengths:
    if picamera_focal_lengths[res] is None:
        # focal length in px = (focal length in mm / sensor width in mm) * image width in px
        focal_length_px_x = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * res[0]
        focal_length_px_y = (FOCAL_LENGTH_MM / SENSOR_HEIGHT_MM) * res[1]
        picamera_focal_lengths[res] = (focal_length_px_x, focal_length_px_y)

#Lets not bother doing this for USB camera's if we don't support it they can't use it

# Colours are in the format BGR
PURPLE = (255,0,215) #Purple
ORANGE = (0,128,255) #Orange
YELLOW = (0,255,255) #Yellow
GREEN = (0,255,0) #Green
RED = (0,0,255) #Red
BLUE = (255,0,0) #Blue
WHITE = (255, 255, 255) #White

#Image post processing
BOUNDING_BOX_THICKNESS = 2

MARKER_ARENA, MARKER_TOKEN, MARKER_BUCKET_SIDE, MARKER_BUCKET_END = 'arena', 'token', 'bucket-side', 'bucket-end'
TOKEN_NONE, TOKEN_ORE, TOKEN_FOOLS_GOLD, TOKEN_GOLD = 'none', 'ore', 'fools-gold', 'gold'

marker_offsets = {
    MARKER_ARENA: 0,
    MARKER_TOKEN: 32,
    MARKER_BUCKET_SIDE: 72,
    MARKER_BUCKET_END: 76,
}

# The numbers here (e.g. `0.25`) are in metres -- the 10/12 is a scaling factor
# so that libkoki gets the size of the 10x10 black/white portion (not including
# the white border), but so that humans can measure sizes including the border.
marker_sizes = {
    MARKER_ARENA: 0.25 * (10.0 / 12),
    MARKER_TOKEN: 0.1 * (10.0 / 12),
    MARKER_BUCKET_SIDE: 0.1 * (10.0 / 12),
    MARKER_BUCKET_END: 0.1 * (10.0 / 12),
}

MarkerInfo = namedtuple("MarkerInfo", "code marker_type token_type offset size bounding_box_colour")
ImageCoord = namedtuple("ImageCoord", "x y")
WorldCoord = namedtuple("WorldCoord", "x y z")
PolarCoord = namedtuple("PolarCoord", "length rot_x rot_y")
Orientation = namedtuple("Orientation", "rot_x rot_y rot_z")
Point = namedtuple("Point", "image world polar")

# Number of markers per group
marker_group_counts = {
    "dev": [(MARKER_ARENA, 24),
            (MARKER_TOKEN, 40),
            (MARKER_BUCKET_SIDE, 4),
            (MARKER_BUCKET_END, 4)],
    "comp": [(MARKER_ARENA, 24),
             (MARKER_TOKEN, 40),
             (MARKER_BUCKET_SIDE, 4),
             (MARKER_BUCKET_END, 4)],
}

def create_marker_lut(counts, zone):  # def create_marker_lut(offset, counts, zone):
    lut = {}
    for marker_type, num_markers in counts:
        for n in range(0, num_markers):
            token_type = TOKEN_NONE
            if marker_type == MARKER_TOKEN:
                if n < 10:
                    token_type = TOKEN_ORE
                    bounding_box_colour = GREEN
                else:
                    token_n = n - 10
                    if int(token_n / 3) == zone:
                        token_type = TOKEN_GOLD
                        bounding_box_colour = YELLOW
                    else:
                        token_type = TOKEN_FOOLS_GOLD
                        bounding_box_colour = RED
            else: #arena marker
                bounding_box_colour = BLUE

            code = marker_offsets[marker_type] + n
            m = MarkerInfo(code=code,
                           marker_type=marker_type,
                           token_type=token_type,
                           offset=n,
                           size=marker_sizes[marker_type],
                           bounding_box_colour = bounding_box_colour)
            lut[code] = m
    return lut


# 0: arena
# ...
# 23: arena
# [24-31 undefined]
# 32: token
# ...
# 71: token
# 72: bucket side
# ...
# 75: bucket side
# 76: bucket end
# ...
# 79: bucket end


marker_luts = {
    "dev": {
        "A": [
            create_marker_lut(marker_group_counts["dev"], 0),
            create_marker_lut(marker_group_counts["dev"], 1),
            create_marker_lut(marker_group_counts["dev"], 2),
            create_marker_lut(marker_group_counts["dev"], 3)
        ],
        "B": [
            create_marker_lut(marker_group_counts["dev"], 0),
            create_marker_lut(marker_group_counts["dev"], 1),
            create_marker_lut(marker_group_counts["dev"], 2),
            create_marker_lut(marker_group_counts["dev"], 3)
        ]
    },
    "comp": {
        "A": [
            create_marker_lut(marker_group_counts["comp"], 0),
            create_marker_lut(marker_group_counts["comp"], 1),
            create_marker_lut(marker_group_counts["comp"], 2),
            create_marker_lut(marker_group_counts["comp"], 3)
        ],
        "B": [
            create_marker_lut(marker_group_counts["comp"], 0),
            create_marker_lut(marker_group_counts["comp"], 1),
            create_marker_lut(marker_group_counts["comp"], 2),
            create_marker_lut(marker_group_counts["comp"], 3)
        ]
    }
}

MarkerBase = namedtuple("Marker", "info timestamp res vertices centre orientation")


class Marker(MarkerBase):
    # noinspection PyUnusedLocal,PyMissingConstructor
    def __init__(self, *a, **kwd):
        # Aliases
        self.dist = self.centre.polar.length
        self.rot_y = self.centre.polar.rot_y


class Timer(object):
    def __enter__(self):
        self.start = time.time()

    def __exit__(self, t, v, tb):
        self.time = time.time() - self.start
        return False


# noinspection PyShadowingNames
class Vision(object):
    def __init__(self, camera_device, lib=None, res=(1296, 736)):
        if lib is not None:
            self.koki = pykoki.PyKoki(lib)
        else:
            self.koki = pykoki.PyKoki()
        
        self._camera_device = camera_device
        
        if camera_device is not None:
            self.camera = self.koki.open_camera(self._camera_device)
        else:
            self.camera = picamera.PiCamera(resolution=res)

        if not isinstance(self.camera, picamera.PiCamera):
            # Lock for the use of the vision
            self.lock = threading.Lock()
            self.lock.acquire()
            self._res = None
            self._buffers = None
            self._streaming = False
            self._set_res(res)
            self._start_camera_stream()
            self.lock.release()

    def __del__(self):
        if not isinstance(self.camera, picamera.PiCamera):
            self._stop_camera_stream()

    def _set_res(self, res):
        """Set the resolution of the camera if different to what we were"""

        if isinstance(self.camera, picamera.PiCamera):
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
        else:
            if res == self._res:
                # Resolution already the requested one
                return

            was_streaming = self._streaming
            if was_streaming:
                self._stop_camera_stream()

            del self.camera
            self.camera = self.koki.open_camera(self._camera_device)
            self.camera.format = self.koki.v4l_create_YUYV_format(*res)

            fmt = self.camera.format.fmt
            width = fmt.pix.width
            height = fmt.pix.height
            actual = (width, height)
            
            if was_streaming:
                self._start_camera_stream() 

        if res != actual:
            raise ValueError("Unsupported image resolution {0} (got: {1})".format(res, actual))

        self._res = actual
        

    def _start_camera_stream(self):
        assert not isinstance(self.camera, picamera.PiCamera)
        self.camera.prepare_buffers(1)
        self.camera.start_stream()
        self._streaming = True

    def _stop_camera_stream(self):
        assert not isinstance(self.camera, picamera.PiCamera)
        self.camera.stop_stream()
        self._streaming = False

    @staticmethod
    def _width_from_code(lut, code):
        if code not in lut:
            # We really want to ignore these...
            return 0.1

        return lut[code].size

    def see(self,
            mode,
            arena,
            res=None,
            stats=False,
            save=True,
            fast_capture=True,
            zone=0,
            bounding_box_enable=False):
                
                
        if isinstance(self.camera, picamera.PiCamera):
            if res is not None and res not in picamera_focal_lengths:
                raise ValueError("Invalid resolution: {}".format(res))
        else:
            if res is not None and res not in usbcamera_focal_lengths:
                raise ValueError("Invalid resolution: {}".format(res))
            self.lock.acquire()
        
        if res is not None:
            self._set_res(res)

        acq_time = time.time()

        timer = Timer()
        times = {}

        with timer:
            if isinstance(self.camera, picamera.PiCamera):
                with picamera.array.PiRGBArray(self.camera) as stream:
                    start = time.time()
                    
                    self.camera.capture(stream, format="bgr", use_video_port=fast_capture)
                    col_image = stream.array
                    image = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
            else:
                frame = self.camera.get_frame()

        times["cam"] = timer.time

        with timer:
            if isinstance(self.camera, picamera.PiCamera):
                if os.path.exists('/media/RobotUSB/collect_images.txt'):
                    filename = "/media/RobotUSB/" + str(int(acq_time)) + ".jpg"
                    cv2.imwrite(filename, image)
                # Create an IplImage header for the image.
                # (width, height), depth, num_channels
                ipl_image = cv2.cv.CreateImageHeader((image.shape[1], image.shape[0]), cv2.cv.IPL_DEPTH_8U, 1)
                # Put the actual image data in the IplImage.
                # The third argument is the row length ("step").
                # Note that pykoki will automatically free `ipl_image` after the markers are obtained from it.
                cv2.cv.SetData(ipl_image, image.tobytes(), image.dtype.itemsize * image.shape[1])
                # Make sure the image data is actually in the IplImage. Don't touch this line!
                ipl_image.tostring()
            else:
                ipl_image = self.koki.v4l_YUYV_frame_to_grayscale_image(frame, *self._res)
        times["manipulation"] = timer.time

        if not isinstance(self.camera, picamera.PiCamera):
            # Lock for the use of the vision
            self.lock = threading.Lock()

        if isinstance(self.camera, picamera.PiCamera):
            params = CameraParams(Point2Df(self.camera.resolution[0] / 2,
                                        self.camera.resolution[1] / 2),
                                Point2Df(*picamera_focal_lengths[self.camera.resolution]),
                                Point2Di(*self.camera.resolution))
        else:
            params = CameraParams(Point2Df(self._res[0] / 2,
                                           self._res[1] / 2),
                                  Point2Df(*usbcamera_focal_lengths[self._res]),
                                  Point2Di(*self._res))

        with timer:
            markers = self.koki.find_markers_fp(ipl_image,
                                                functools.partial(self._width_from_code, marker_luts[mode][arena]),
                                                params)
        times["find_markers"] = timer.time

        srmarkers = []

        usb_log = os.path.exists("/media/RobotUSB/log_markers.txt")

        if markers and usb_log:
            logfile = open("/media/RobotUSB/" + str(int(acq_time)) + ".txt", "w")

        for m in markers:
            if usb_log:
                # noinspection PyUnboundLocalVariable
                logfile.write("code: {}, distance: {}, rot_x: {}, rot_y: {}\n".format(
                    m.code, round(m.distance, 3), round(m.bearing.x, 3), round(m.bearing.y, 3)))
            if m.code not in marker_luts[mode][arena][zone]:
                # Ignore other sets of codes
                if usb_log:
                    logfile.write("(last marker not part of competition, ignoring)\n")
                continue

            info = marker_luts[mode][arena][zone][int(m.code)]

        times["logging"] = timer.time
        
        for m in markers:
            if bounding_box_enable:
                bounding_box_colour = info.bounding_box_colour
                for i in range(0, len(m.vertices)):
                    vertex1 = m.vertices[i].image
                    
                    try:
                        vertex2 = m.vertices[i+1].image
                    except IndexError:
                        vertex2 = m.vertices[0].image

                    point1 = (int(vertex1.x), int(vertex1.y))
                    point2 = (int(vertex2.x), int(vertex2.y))
                    
                    cv2.line(col_image, point1, point2, bounding_box_colour, BOUNDING_BOX_THICKNESS)
        
        times["bounding_boxes"] = timer.time
        
        for m in markers:
            vertices = []
            for v in m.vertices:
                #Append to the list
                vertices.append(Point(image=ImageCoord(x=v.image.x,
                                                       y=v.image.y),
                                      world=WorldCoord(x=v.world.x,
                                                       y=v.world.y,
                                                       z=v.world.z),
                                      # libkoki does not yet provide these coords
                                      polar=PolarCoord(0, 0, 0)))

            num_quarter_turns = int(m.rotation_offset / 90)
            num_quarter_turns %= 4

            vertices = vertices[num_quarter_turns:] + vertices[:num_quarter_turns]

            centre = Point(image=ImageCoord(x=m.centre.image.x,
                                            y=m.centre.image.y),
                           world=WorldCoord(x=m.centre.world.x,
                                            y=m.centre.world.y,
                                            z=m.centre.world.z),
                           polar=PolarCoord(length=m.distance,
                                            rot_x=m.bearing.x,
                                            rot_y=m.bearing.y))

            orientation = Orientation(rot_x=m.rotation.x,
                                      rot_y=m.rotation.y,
                                      rot_z=m.rotation.z)

            marker = Marker(info=info,
                            timestamp=acq_time,
                            res=res,
                            vertices=vertices,
                            centre=centre,
                            orientation=orientation)
            srmarkers.append(marker)

        if save:
            cv2.imwrite("/tmp/colimage.jpg", col_image)
        
        if markers and usb_log:
            logfile.close()

        if not isinstance(self.camera, picamera.PiCamera):
            self.koki.image_free(ipl_image)

        times["output_properties"] = timer.time
        if stats:
            return srmarkers, times

        return srmarkers
