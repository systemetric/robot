import apriltags3_py as AT
import cv2

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
