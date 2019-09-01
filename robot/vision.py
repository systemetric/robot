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

picamera_focal_lengths = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

usbcamera_focal_lengths = {  # fx, fy tuples
    (1920, 1440): (1393, 1395),
    (1920, 1088): (2431, 2431),
    (1296, 976): (955, 955),
    (1296, 736): (962, 962),
    (640, 480): (463, 463),
}

pi_cam_resolution = (1296, 730)
focal_length = (962, 962)

# Colours are in the format BGR
PURPLE = (255,0,215) #Purple
ORANGE = (0,128,255) #Orange
YELLOW = (0,255,255) #Yellow
GREEN = (0,255,0) #Green
RED = (0,0,255) #Red
BLUE = (255,0,0) #Blue
WHITE = (255, 255, 255) #White

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

FrameResult = namedtuple("FrameResult", "time frame result")
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
    bounding_box_colour = GREEN
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
                           size=marker_sizes[marker_type],
                           bounding_box_colour=bounding_box_colour)
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

mode = "dev"
arena = "A"
zone=0

class FrameProcessor(threading.Thread):
    """
    Turns a frame into a numpy array -> cv::mat -> iplimage passes it to
    koki then tries to store the result its owner.
    
    Each instance of this class runs in a seperate thread and is controled
    by the StreamAnalyzer class.
    """
    def __init__(self, owner, lib=None, preprocessing=None, use_usb_cam=False):
        super(FrameProcessor, self).__init__()
        
        self.stream = io.BytesIO()
        self.owner = owner
        self.preprocessing = preprocessing
        
        self.camera = self.owner.camera
        
        self.use_usb_cam = use_usb_cam
        self.usb_cam_frame = None
        
        self.fast_capture = True #TODO: add option to disable this
        
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
            return 0.1

    def run(self):
        """
        The main loop for the processor, waits to be told a frame has been
        writen to the buffer using the new_image_event. Then it takes the
        frame applying different processing techniques. As we are multithreaded
        waiting on a long IO event does not cost us much in fps only latency.
        
        Once we have the frame, we convert into a form koki can use call
        koki and update the last_analyzed_frame parameter in the controlling
        class.
        
        Then the thread is returned to the pool to await another frame
        """
        while not self._stop_event.is_set():
            if self.new_image_event.wait(1):
                try:
                    start_work_time = time.time()
                    
                    if self.use_usb_cam:
                        colour_image = self.usb_cam_frame
                        image = cv2.cvtColor(colour_image, cv2.COLOR_BGR2GRAY)
                    else:
                        self.stream.seek(0)

                        if self.preprocessing == None:
                            data = numpy.fromstring(self.stream.getvalue(), dtype=numpy.uint8)
                            image = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_GRAYSCALE)
                            colour_image = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
                        
                        elif self.preprocessing == "video-denoise":
                            colour_image = Image.open(self.stream)
                            image = cv2.cvtColor(numpy.array(colour_image), cv2.COLOR_RGB2GRAY)
                            
                        elif self.preprocessing == "picture-denoise":
                            with self.owner.lock:
                                with picamera.array.PiRGBArray(self.owner.camera) as stream:
                                    self.owner.camera.capture(stream, format="bgr", use_video_port=self.fast_capture)
                                    colour_image = stream.array
                                    image = cv2.cvtColor(colour_image, cv2.COLOR_BGR2GRAY)
                        else:
                            raise Error("Invalid image processor state, this should never happen")
                        # See: https://picamera.readthedocs.io/en/release-1.13/recipes2.html#rapid-capture-and-processing
                                            
                    # Create an IplImage header for the image.
                    # (width, height), depth, num_channels
                    ipl_image = cv2.cv.CreateImageHeader((image.shape[1], image.shape[0]), cv2.cv.IPL_DEPTH_8U, 1)
                    # Put the actual image data in the IplImage.
                    # The third argument is the row length ("step").
                    # Note that pykoki will automatically free `ipl_image` after the markers are obtained from it.
                    cv2.cv.SetData(ipl_image, image.tobytes(), image.dtype.itemsize * image.shape[1])
                    # Make sure the image data is actually in the IplImage. Don't touch this line!
                    ipl_image.tostring()
                    
                    if self.use_usb_cam:
                        params = CameraParams(Point2Df(self.owner._res[0] / 2,
                        self.owner._res[1] / 2),
                        Point2Df(*usbcamera_focal_lengths[self.owner._res]),
                        Point2Di(*self.owner._res))
                    else:
                        params = CameraParams(Point2Df(self.camera.resolution[0] / 2,
                        self.camera.resolution[1] / 2),
                        Point2Df(*picamera_focal_lengths[self.camera.resolution]),
                        Point2Di(*self.camera.resolution))
                                
                    markers = self.koki.find_markers_fp(ipl_image,
                                    functools.partial(self._width_from_code, marker_luts[mode][arena]),
                                    params)
                                                                                            
                    # Lock the thread so that the parent varible doesn't change under our feet
                    # Not sure if this is needed but it doesn't cost us much
                    with self.owner.lock:
                        if self.owner.last_analyzed_frame.time < time.time():
                            self.owner.last_analyzed_frame = FrameResult(
                                time = time.time(),
                                frame = colour_image,
                                result = markers)
                            self.owner.owner.new_analysis_for_user.set()
                                    
                finally:
                    if not self.use_usb_cam:
                        self.stream.seek(0)
                        self.stream.truncate()
                    
                    self.new_image_event.clear()
                    with self.owner.lock:
                        self.owner.pool.append(self)
                        
        #We have been asked to stop and so we will exit here
        self.stopped()
        
class PreprocessingSetter(object):
    """
    Alows setting/getting of all the threads preprocessing modes from a
    signle parameter of the StreamAnalyzer class 
    """
    def __init__(self, stream_analyzer):
        self.stream_analyzer = stream_analyzer
    
    def __setitem__(self, value):
        if value not in [None, "video-denoise", "picture-denoise"]:
            raise ValueError("Invalid Preprossing mode {}".format(value))
        
        with self.stream_analyzer.lock:
            for processor in self.stream_analyzer.pool:
                processor.preprocessing = value
                
    def __getitem__(self, value):
        result = []
        
        with self.stream_analyzer.lock:
            for processor in self.stream_analyzer.pool:
                result.append(processor.preprocessing)
        
        return result

        
class StreamAnalyzer(object):
    """
    Schedules threads with frames comming in from a stream storing the
    most recent analysis.
    """
    def __init__(self, owner, camera, libkoki_path, thread_count, use_usb_cam, res):
        self.camera = camera
        self.owner = owner
        
        self._res = res
                    
        # Construct a pool of frame processors
        self.lock = threading.Lock()            
        self.pool = [FrameProcessor(self, libkoki_path, use_usb_cam=use_usb_cam) for i in range(thread_count)]
            
        self.use_usb_cam = use_usb_cam
        self.processor = None
        self.thread_count = thread_count
        
        # fake the time of the last analyzed frame as the time the 
        # program started.
        self.last_analyzed_frame = FrameResult(
                            time = time.time(),
                            frame = None,
                            result = None)

            
    def write(self, buf):
        """
        When a frame is writen from the camera we set the current processor
        going, and attempt to get a new one from the pool however the pool maybe
        empty in which case we have to skip the frame. 
        
        We then start writing the buffer to the next processor, which we
        know has finished when the next frame comes in
        """
        if self.use_usb_cam:
            if self.processor:
                self.processor.usb_cam_frame = buf
                self.processor.new_image_event.set()

            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else: 
                    #No avalible processor
                    self.processor = None
        else:
            if buf.startswith(b'\xff\xd8'):
                if self.processor:
                    self.processor.new_image_event.set()
                    
                with self.lock:
                    if self.pool:
                        self.processor = self.pool.pop()
                    else: 
                        #No avalible processor
                        self.processor = None
            
            #Pass the buffer to current processor 
            if self.processor:
                self.processor.stream.write(buf)

    def return_processor_to_pool(self):
        """
        Returns the processor to the pool
        """
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        

    def flush(self):
        """
        This method is called by the camera when it stops recording
        so we add the current processor back to the pool
        """
        self.return_processor_to_pool()
        

                    
    def shutdown(self):
        """
        Stops and returns threads to the pool. Joining them as we go.
        """        
        self.return_processor_to_pool()
        
        running_threads = self.thread_count
        
        while running_threads:
            with self.lock:
                try:
                    proc = self.pool.pop()
                    print "popped thread: ", proc
                    proc.stop()
                    proc.join()
                    running_threads -= 1
                except IndexError:
                    time.sleep(0.1)


class VisionController(object):
    """
    The controller class for vision. Contains an instance of the 
    StreamAnalyzer class (the scheduler for threads) and camera classes.
    It's purpose is to set everything up for the StreamAnalyzer class and
    to feed it frames from the camera's
    
    This probally could be merged with the ProcessOuput class by using self
    as the writeable object. Not sure if that is more or less readable.
    """
    def __init__(self, res, thread_count, use_usb_cam=False):
        
        # Find libkoki.so:
        libpath = "/home/pi/libkoki/lib"
        if "LD_LIBRARY_PATH" in os.environ:
            for d in os.environ["LD_LIBRARY_PATH"].split(":"):
                l = glob.glob("%s/libkoki.so*" % os.path.abspath(d))
                if len(l):
                    libpath = os.path.abspath(d)
                    break
        
        self.use_usb_cam = use_usb_cam
        
        if self.use_usb_cam:
            self.camera = cv2.VideoCapture(1)
        else:
            self.camera = picamera.PiCamera(resolution=res)
        
        self.stream_analyzer = StreamAnalyzer(self, self.camera, libpath, thread_count, use_usb_cam, res)        
        self.preprocessing = PreprocessingSetter(self.stream_analyzer)
        
        self.new_analysis_for_user = threading.Event() 
        self.new_analysis_for_user.clear()
        
        self.done = False
        
        try:
            self.analyze_thread = threading.Thread(target=self.record_to_analyzer)
            self.analyze_thread.start()
        except:
            print "Error: Failed starting vision analyzer"

        
    def record_to_analyzer(self):
        if self.use_usb_cam:
            while not self.done:
                ret, frame = self.camera.read() #Assume that it worked
                self.stream_analyzer.write(frame)
        else:
            self.camera.start_recording(self.stream_analyzer, format='mjpeg')
            while not self.done:
                self.camera.wait_recording(1)
        
            
    def see(self,
            save=True,
            stats=None,
            bounding_box_enable=True,
            zone=0,
            mode="dev",
            arena="A"):
        """
        The function which interacts with the user to return the markers
        which the robot last saw in a useful form to the user
        Parameters: 
        save  - BOOL Weather the robot should save a copy of what it saw to /tmp
        stats - DICT Passing an empty dictionary {} will be filled during
                     the call with times for each step of the pipeline
        zone  - INT  What zone the robot will for the LUTs
        bounding_box_enable - allows or prevents bounding boxes from being drawn
        
        """
        timer = Timer()
        times = {}
        
        res = pi_cam_resolution #Set the res until we have proper resolution switching   
        
        # If there is no analys is for the user then we wait 1/20s        
        with timer:
            while not self.new_analysis_for_user.wait(0.05):
                pass
                
            self.new_analysis_for_user.clear()
            markers = self.stream_analyzer.last_analyzed_frame.result
            acq_time = self.stream_analyzer.last_analyzed_frame.time
            colour_frame = self.stream_analyzer.last_analyzed_frame.frame
                        
        times["acq"] = timer.time

        # Post asignment 
        with timer:
            robocon_markers = []

            for m in markers:
                if m.code not in marker_luts[mode][arena][zone]:
                    print "WARNING: Libkoki detected marker not found in competition"
                    print "try using better lighting"
                    print "IGNORING MARKER", m
                    continue
        
                
                info = marker_luts[mode][arena][zone][int(m.code)]
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
                robocon_markers.append(marker)
        
        times["post-processing"] = timer.time
        
        if bounding_box_enable:
            with timer:
                #Should this include the markers not found in the LUT?
                for m in robocon_markers:
                    bounding_box_colour = m.info.bounding_box_colour
                    #get the image cords of the diganoally oposite vertecies
                    vertex1 = (int(m.vertices[0].image.x), int(m.vertices[0].image.y))
                    vertex3 = (int(m.vertices[2].image.x), int(m.vertices[2].image.y))
                    cv2.rectangle(colour_frame, vertex1, vertex3, bounding_box_colour, BOUNDING_BOX_THICKNESS)
            times["bounding_box"] = timer.time
    
        
        if save:
            with timer:
                cv2.imwrite("/tmp/colimage.jpg", colour_frame)

            times["save"] = timer.time
        
        if stats:
            stats = times
        
        return robocon_markers

            
    def tidyUp(self):    
        self.done = True
        self.stream_analyzer.shutdown()
        self.analyze_thread.join()
        self.camera.stop_recording()
        
            
    def __del__(self):
        self.tidyUp()
        
