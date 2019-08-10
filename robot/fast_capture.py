import io
import time
import threading
import picamera

import functools
import os
import threading
import time
from collections import namedtuple

from PIL import Image
import numpy
import cv2
import pykoki
# noinspection PyUnresolvedReferences
import picamera.array  # Required, see <https://picamera.readthedocs.io/en/latest/api_array.html>
from pykoki import CameraParams, Point2Df, Point2Di

picamera_focal_lengths = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

pi_cam_resolution = (1296, 736)
focal_length = (962, 962)

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

MarkerInfo = namedtuple("MarkerInfo", "code marker_type token_type offset size")
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
                else:
                    token_n = n - 10
                    if int(token_n / 3) == zone:
                        token_type = TOKEN_GOLD
                    else:
                        token_type = TOKEN_FOOLS_GOLD

            code = marker_offsets[marker_type] + n
            m = MarkerInfo(code=code,
                           marker_type=marker_type,
                           token_type=token_type,
                           offset=n,
                           size=marker_sizes[marker_type])
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

mode = "dev"
arena = "A"


class ImageProcessor(threading.Thread):
    def __init__(self, owner, lib=None):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        
        if lib is not None:
            self.koki = pykoki.PyKoki(lib)
        else:
            self.koki = pykoki.PyKoki()       
        
        self.start()

    @staticmethod
    def _width_from_code(lut, code):
        if code not in lut:
            # We really want to ignore these...
            return 0.1

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    #Image.open(self.stream)
                    self.stream.seek(0)
                    pil_image = Image.open(self.stream)
                    image = cv2.cvtColor(numpy.array(pil_image), cv2.COLOR_RGB2GRAY)
                    #image = cv2.cvtColor(picamera.array.PiRGBArray, cv2.COLOR_BGR2GRAY)
                    
                    # Create an IplImage header for the image.
                    # (width, height), depth, num_channels
                    ipl_image = cv2.cv.CreateImageHeader((image.shape[1], image.shape[0]), cv2.cv.IPL_DEPTH_8U, 1)
                    # Put the actual image data in the IplImage.
                    # The third argument is the row length ("step").
                    # Note that pykoki will automatically free `ipl_image` after the markers are obtained from it.
                    cv2.cv.SetData(ipl_image, image.tobytes(), image.dtype.itemsize * image.shape[1])
                    # Make sure the image data is actually in the IplImage. Don't touch this line!
                    ipl_image.tostring()

                    params = CameraParams(Point2Df(pi_cam_resolution[0] / 2,
                                        pi_cam_resolution[1] / 2),
                                Point2Df(*focal_length),
                                Point2Di(*pi_cam_resolution))
                                
                    markers = self.koki.find_markers_fp(ipl_image,
                                    functools.partial(self._width_from_code, marker_luts[mode][arena]),
                                    params)
                                    
                    print(markers)
                    # Set done to True if you want the script to terminate
                    # at some point
                    #self.owner.done=True
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)

class ProcessOutput(object):
    def __init__(self):
        self.done = False
        self.processed_frames = 0
        
        # Find libkoki.so:
        libpath = None
        if "LD_LIBRARY_PATH" in os.environ:
            for d in os.environ["LD_LIBRARY_PATH"].split(":"):
                l = glob.glob("%s/libkoki.so*" % os.path.abspath(d))

                if len(l):
                    libpath = os.path.abspath(d)
                    break
                    
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()            
        if libpath is None:
           self.pool = [ImageProcessor(self, "/home/pi/libkoki/lib") for i in range(4)]
        else:
           self.pool = [ImageProcessor(self, libpath) for i in range(4)]
            
        self.processor = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    self.processor = None
        if self.processor:
            self.processed_frames += 1
            print self.processed_frames
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True
            proc.join()

with picamera.PiCamera(resolution=pi_cam_resolution) as camera:
    #camera.start_preview()
    start = time.time()
    try:
        print "Camera init waiting 2 seconds for it to settle"
        time.sleep(2)
        print "Running"
        output = ProcessOutput()
        camera.start_recording(output, format='mjpeg')
        while not output.done:
            camera.wait_recording(1)
    finally:
        time_taken = time.time() - start
        print "ending after: " + time_taken
        camera.stop_recording()
