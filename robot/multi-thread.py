import io
import time
import threading
import picamera

import functools
import os
import threading
from collections import namedtuple

from PIL import Image
import numpy
import cv2
import pykoki
from pykoki import CameraParams, Point2Df, Point2Di
import picamera.array  # Required, see <https://picamera.readthedocs.io/en/latest/api_array.html>


thread_count = 4

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

FrameResult = namedtuple("FrameResult", "time frame_pointer result")
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

class camera():
    def __init__(self):
        pass


class FrameProcessor(threading.Thread):
    def __init__(self, owner, lib=None, preprocessing="picture-denoise"):
        super(FrameProcessor, self).__init__()
        
        self.stream = io.BytesIO()
        self.owner = owner
        self.preprocessing = preprocessing
        
        #create events that let us control this processor
        self.new_image_event = threading.Event()
        self._stop_event = threading.Event()
        
        if lib is None:
            self.koki = pykoki.PyKoki()
        else:
            self.koki = pykoki.PyKoki(lib)       
        
        self.start()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @staticmethod
    def _width_from_code(lut, code):
        if code not in lut:
            # We really want to ignore these...
            return 0.1

    def run(self):
        while not self._stop_event.is_set():
            # Wait for an image to be written to the stream
            if self.new_image_event.wait(1):
                try:
                    start_work_time = time.time()
                    
                    # Move to the start of the stream
                    self.stream.seek(0)
                    
                    if self.preprocessing == None:
                        # Construct a numpy array from the last video frame in the stream
                        data = numpy.fromstring(self.stream.getvalue(), dtype=numpy.uint8)
                        # "Decode" the image from the array converting to gray
                        image = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_GRAYSCALE)
                        
                    elif self.preprocessing == "picture-denoise":
                        #Should use YUV instead of BGR so the GPU has to do less work
                        with picamera.array.PiRGBArray(self.camera) as stream:
                            self.camera.capture(stream, format="bgr", use_video_port=fast_capture)
                            image = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
                    # TODO: test if this is actually more accurate or can see further
                    """
                    elif self.preprocessing == "video-denoise":
                        pil_image = Image.open(self.stream)
                        image = cv2.cvtColor(numpy.array(pil_image), cv2.COLOR_RGB2GRAY)
                    """
                    
                                            
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
                                
                    #call koki on the image
                    markers = self.koki.find_markers_fp(ipl_image,
                                    functools.partial(self._width_from_code, marker_luts[mode][arena]),
                                    params)
                                    
                    print markers
                    
                    # Lock the thread so that the parent varible doesn't change under our feet
                    # Not sure if this is needed but it doesn't cost us much
                    with self.owner.lock:
                        #Only add the frame if the last one added was older
                        if self.owner.last_analyzed_frame.time < time.time():
                            self.owner.last_analyzed_frame = FrameResult(
                                time = time.time(),
                                frame_pointer = image,
                                result = markers)
                                    
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.new_image_event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)
                        
        #We have been asked to stop and so we will exit here
        self.stopped()    

class StreamAnalyzer(object):
    def __init__(self, camera, libkoki_path):
        self.camera = camera
                    
        # Construct a pool of image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()            
        self.pool = [FrameProcessor(self, libkoki_path) for i in range(thread_count)]
            
        self.processor = None
        self.processed_frames = 0
        
        # fake the time of the last analyzed frame as the time the 
        # program started. This feels a bit hacky but works.
        self.last_analyzed_frame = FrameResult(
                            time = time.time(),
                            frame_pointer = None,
                            result = None)

        
    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.new_image_event.set()
                self.processed_frames += 1
                print "processed frames: ", self.processed_frames
                
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    self.processor = None
        
        #Pass the buffer to current processor 
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        running_threads = thread_count
        print "output.flush() called"
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        
        # Now, empty the pool, joining each thread as we go.
        # We keep track of how many threads are left so that we don't
        # leave any still running before the main thread exits
        while running_threads:
            with self.lock:
                try:
                    proc = self.pool.pop()
                    print "popped thread: ", proc
                    proc.stop()
                    proc.join()
                    running_threads -= 1
                except IndexError:
                    print "waiting for threads to return to pool"
                    #Wait for the threads to return themselves to the pool
                    time.sleep(0.1)


class VisionController():
    """
    The controller class for vision. Contains an instance of the 
    StreamAnalyzer class (the scheduler for threads) and camera classes.
    It's purpose is to set everything up for the StreamAnalyzer class and
    to feed it frames from the camera's
    
    This probally could be merged with the ProcessOuput class by using self
    as the writeable object. Not sure if that is more or less readable.
    """
    def __init__(self, res):
        
        # Find libkoki.so:
        libpath = "/home/pi/libkoki/lib"
        if "LD_LIBRARY_PATH" in os.environ:
            for d in os.environ["LD_LIBRARY_PATH"].split(":"):
                l = glob.glob("%s/libkoki.so*" % os.path.abspath(d))
                if len(l):
                    libpath = os.path.abspath(d)
                    break
        
        self.camera = picamera.PiCamera(resolution=res)
        self.stream_analyzer = StreamAnalyzer(self.camera, libpath)
        
        self.new_analysis_for_user = threading.Event() 
        self.new_analysis_for_user.clear()
        
        self.done = False
        
        try:
            self.analyze_thread = threading.Thread(target=self.record_to_analyzer)
            self.analyze_thread.start()
        except:
            print "Error: Failed starting analyzer"

        
    def record_to_analyzer(self):
        print "record_to_analyzer has been called"
        print "attempting to start recording"
        self.camera.start_recording(self.stream_analyzer, format='mjpeg')
        print "camera has started recording"
        while not self.done:
            self.camera.wait_recording(1)
        #finally:
        self.camera.stop_recording()
                
    def see(self):
        if self.new_analysis_for_user.wait(1):
            self.new_analysis_for_user.clear()
            return self.stream_analyzer.last_analyzed_frame
        else:
            raise IOError("StreamAnalyzer failed to return new analysis")
            
        
    def __del__(self):
        self.done = True
        self.analyze_thread.join()
        
myVisionController = VisionController(pi_cam_resolution)
print myVisionController.see()
        
