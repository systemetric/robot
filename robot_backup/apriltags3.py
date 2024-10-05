#!/usr/bin/env python
"""Python wrapper for C version of apriltags. This program creates two
classes that are used to detect apriltags and extract information from
them. Using this module, you can identify all apriltags visible in an
image, and get information about the location and orientation of the
tags.

This module is both python 2 and 3 compatiable.

Forked from: <https://github.com/duckietown/apriltags3-py>
Added support for polar cords and variable sized markers
"""
from __future__ import division
from __future__ import print_function

import ctypes
import os
import itertools
from collections import namedtuple

import numpy as np
import scipy.spatial.transform as transform

from robot.marker_setup.markers import MARKER


######################################################################
# Types

Coords = namedtuple("Coords", ("x", "y", "z"))

######################################################################

# pylint: disable=R0903


class _ImageU8(ctypes.Structure):
    '''Wraps image_u8 C struct.'''
    _fields_ = [
        ('width', ctypes.c_int),
        ('height', ctypes.c_int),
        ('stride', ctypes.c_int),
        ('buf', ctypes.POINTER(ctypes.c_uint8))
    ]


class _Matd(ctypes.Structure):
    '''Wraps matd C struct.'''
    _fields_ = [
        ('nrows', ctypes.c_int),
        ('ncols', ctypes.c_int),
        ('data', ctypes.c_double*1),
    ]


class _ZArray(ctypes.Structure):
    '''Wraps zarray C struct.'''
    _fields_ = [
        ('el_sz', ctypes.c_size_t),
        ('size', ctypes.c_int),
        ('alloc', ctypes.c_int),
        ('data', ctypes.c_void_p)
    ]


class _ApriltagFamily(ctypes.Structure):
    '''Wraps apriltag_family C struct.'''
    _fields_ = [
        ('ncodes', ctypes.c_uint32),
        ('codes', ctypes.POINTER(ctypes.c_uint64)),
        ('width_at_border', ctypes.c_int),
        ('total_width', ctypes.c_int),
        ('reversed_border', ctypes.c_bool),
        ('nbits', ctypes.c_uint32),
        ('bit_x', ctypes.POINTER(ctypes.c_uint32)),
        ('bit_y', ctypes.POINTER(ctypes.c_uint32)),
        ('h', ctypes.c_int32),
        ('name', ctypes.c_char_p),
    ]


class _ApriltagDetection(ctypes.Structure):
    '''Wraps apriltag_detection C struct.'''
    _fields_ = [
        ('family', ctypes.POINTER(_ApriltagFamily)),
        ('id', ctypes.c_int),
        ('hamming', ctypes.c_int),
        ('decision_margin', ctypes.c_float),
        ('H', ctypes.POINTER(_Matd)),
        ('c', ctypes.c_double*2),
        ('p', (ctypes.c_double*2)*4)
    ]


class _ApriltagDetector(ctypes.Structure):
    '''Wraps apriltag_detector C struct.'''
    _fields_ = [
        ('nthreads', ctypes.c_int),
        ('quad_decimate', ctypes.c_float),
        ('quad_sigma', ctypes.c_float),
        ('refine_edges', ctypes.c_int),
        ('decode_sharpening', ctypes.c_double),
        ('debug', ctypes.c_int)
    ]


class _ApriltagDetectionInfo(ctypes.Structure):
    '''Wraps apriltag_detection_info C struct.'''
    _fields_ = [
        ('det', ctypes.POINTER(_ApriltagDetection)),
        ('tagsize', ctypes.c_double),
        ('fx', ctypes.c_double),
        ('fy', ctypes.c_double),
        ('cx', ctypes.c_double),
        ('cy', ctypes.c_double)
    ]


class _ApriltagPose(ctypes.Structure):
    '''Wraps apriltag_pose C struct.'''
    _fields_ = [
        ('R', ctypes.POINTER(_Matd)),
        ('t', ctypes.POINTER(_Matd))
    ]

######################################################################


def _ptr_to_array2d(datatype, ptr, rows, cols):
    array_type = (datatype*cols)*rows
    array_buf = array_type.from_address(ctypes.addressof(ptr))
    return np.ctypeslib.as_array(array_buf, shape=(rows, cols))


def _image_u8_get_array(img_ptr):
    return _ptr_to_array2d(ctypes.c_uint8,
                           img_ptr.contents.buf.contents,
                           img_ptr.contents.height,
                           img_ptr.contents.stride)


def _matd_get_array(mat_ptr):
    return _ptr_to_array2d(ctypes.c_double,
                           mat_ptr.contents.data,
                           int(mat_ptr.contents.nrows),
                           int(mat_ptr.contents.ncols))


def zarray_get(za, idx, ptr):  # pylint: disable=C0321
    """"
    memcpy(p, &za->data[idx*za->el_sz], za->el_sz);

    p                           = ptr
    za->el_sz                   = za.contents.el_sz
    &za->data[idx*za->el_sz]    = za.contents.data+idx*za.contents.el_sz
    """
    ctypes.memmove(ptr, za.contents.data+idx *
                   za.contents.el_sz, za.contents.el_sz)


######################################################################

class Detection(object):
    """Combined pythonic wrapper for apriltag_detection and apriltag_pose"""

    def __init__(self):
        self.tag_family = None
        self.tag_id = None
        self.hamming = None
        self.decision_margin = None
        self.homography = None
        self.center = None
        self.corners = None
        self.pose_R = None
        self.pose_T = None
        self.pose_err = None
        self.dist = None
        self.bearing = None
        self.rotation = None

    def __str__(self):
        return('Detection object:' +
               '\ntag_family = ' + str(self.tag_family) +
               '\ntag_id = ' + str(self.tag_id) +
               '\nhamming = ' + str(self.hamming) +
               '\ndecision_margin = ' + str(self.decision_margin) +
               '\nhomography = ' + str(self.homography) +
               '\ncenter = ' + str(self.center) +
               '\ncorners = ' + str(self.corners) +
               '\npose_R = ' + str(self.pose_R) +
               '\npose_T = ' + str(self.pose_T) +
               '\npose_err = ' + str(self.pose_err) +
               '\nself.dist = ' + str(self.dist) +
               '\nself.rotation = ' + str(self.rotation) +
               '\nself.bearing = ' + str(self.bearing)+'\n')

    def __repr__(self):
        return self.__str__()


######################################################################

class Detector(object):
    """
    Pythonic wrapper for apriltag_detector.
    families:           Tag families, separated with a space, default: tag36h11

    nthreads:           Number of threads, default: 1

    quad_decimate:      Detection of quads can be done on a lower-resolution
                        image, improving speed at a cost of pose accuracy and a
                        light decrease in detection rate. Decoding the binary
                        payload is still done at full resolution, default: 2.0

    quad_sigma:         What Gaussian blur should be applied to the segmented
                        image (used for quad detection?)  Parameter is the
                        standard deviation in pixels.  Very noisy images
                        benefit from non-zero values (e.g. 0.8), default:  0.0

    refine_edges:       When non-zero, the edges of the each quad are adjusted
                        to "snap to" strong gradients nearby. This is useful
                        when decimation is employed, as it can increase the
                        quality of the initial quad estimate substantially.
                        Generally recommended to be on (1). Very
                        computationally inexpensive. Option is ignored if
                        quad_decimate = 1, default: 1

    decode_sharpening:  How much sharpening should be done to decoded images?
                        This can help decode small tags but may or may not help
                        in odd lighting conditions or low light conditions,
                        default = 0.25

    searchpath:         Where to look for the Apriltag 3 library, must be a
                        itterable, default: ('apriltags')

    debug:              If 1, will save debug images. Runs very slow,
                        default: 0
    """

    def __init__(self,
                 families='tag36h11',
                 nthreads=1,
                 quad_decimate=2.0,
                 quad_sigma=0.0,
                 refine_edges=1,
                 decode_sharpening=0.25,
                 debug=0,
                 searchpath=('apriltags')):

        # Parse the parameters
        self.params = dict()
        self.params['families'] = families.split()
        self.params['nthreads'] = nthreads
        self.params['quad_decimate'] = quad_decimate
        self.params['quad_sigma'] = quad_sigma
        self.params['refine_edges'] = refine_edges
        self.params['decode_sharpening'] = decode_sharpening
        self.params['debug'] = debug

        # detect OS to get extension for DLL
        uname0 = os.uname()[0]
        if uname0 == 'Darwin':
            extension = '.dylib'
        else:
            extension = '.so'

        filename = 'libapriltag'+extension

        self.libc = None
        self.tag_detector = None
        self.tag_detector_ptr = None

        for path in searchpath:
            relpath = os.path.join(path, filename)
            if os.path.exists(relpath):
                self.libc = ctypes.CDLL(relpath)
                break

        # if full path not found just try opening the raw filename;
        # this should search whatever paths dlopen is supposed to
        # search.
        if self.libc is None:
            self.libc = ctypes.CDLL(filename)

        if self.libc is None:
            raise RuntimeError('could not find DLL named ' + filename)

        # create the c-_apriltag_detector object
        self.libc.apriltag_detector_create.restype = ctypes.POINTER(
            _ApriltagDetector)
        self.tag_detector_ptr = self.libc.apriltag_detector_create()

        # create the family
        self.libc.apriltag_detector_add_family_bits.restype = None
        self.tag_families = dict()
        if 'tag16h5' in self.params['families']:
            self.libc.tag16h5_create.restype = ctypes.POINTER(_ApriltagFamily)
            self.tag_families['tag16h5'] = self.libc.tag16h5_create()
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tag16h5'], 2)
        elif 'tag25h9' in self.params['families']:
            self.libc.tag25h9_create.restype = ctypes.POINTER(_ApriltagFamily)
            self.tag_families['tag25h9'] = self.libc.tag25h9_create()
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tag25h9'], 2)
        elif 'tag36h11' in self.params['families']:
            self.libc.tag36h11_create.restype = ctypes.POINTER(_ApriltagFamily)
            self.tag_families['tag36h11'] = self.libc.tag36h11_create()
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tag36h11'], 2)
        elif 'tagCircle21h7' in self.params['families']:
            self.libc.tagCircle21h7_create.restype = ctypes.POINTER(
                _ApriltagFamily)
            self.tag_families['tagCircle21h7'] = self.libc.tagCircle21h7_create()
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tagCircle21h7'], 2)
        elif 'tagCircle49h12' in self.params['families']:
            self.libc.tagCircle49h12_create.restype = ctypes.POINTER(
                _ApriltagFamily)
            self.tag_families['tagCircle49h12'] = self.libc.tagCircle49h12_create()
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tagCircle49h12'], 2)
        elif 'tagCustom48h12' in self.params['families']:
            self.libc.tagCustom48h12_create.restype = ctypes.POINTER(
                _ApriltagFamily)
            self.tag_families['tagCustom48h12'] = self.libc.tagCustom48h12_create()
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tagCustom48h12'], 2)
        elif 'tagStandard41h12' in self.params['families']:
            self.libc.tagStandard41h12_create.restype = ctypes.POINTER(
                _ApriltagFamily)
            self.tag_families['tagStandard41h12'] = self.libc.tagStandard41h12_create(
            )
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tagStandard41h12'], 2)
        elif 'tagStandard52h13' in self.params['families']:
            self.libc.tagStandard52h13_create.restype = ctypes.POINTER(
                _ApriltagFamily)
            self.tag_families['tagStandard52h13'] = self.libc.tagStandard52h13_create(
            )
            self.libc.apriltag_detector_add_family_bits(
                self.tag_detector_ptr, self.tag_families['tagStandard52h13'], 2)
        else:
            raise Exception(
                'Unrecognized tag family name. Use e.g. \'tag36h11\'.\n')

        # configure the parameters of the detector
        self.tag_detector_ptr.contents.nthreads = int(self.params['nthreads'])
        self.tag_detector_ptr.contents.quad_decimate = float(
            self.params['quad_decimate'])
        self.tag_detector_ptr.contents.quad_sigma = float(
            self.params['quad_sigma'])
        self.tag_detector_ptr.contents.refine_edges = int(
            self.params['refine_edges'])
        self.tag_detector_ptr.contents.decode_sharpening = int(
            self.params['decode_sharpening'])
        self.tag_detector_ptr.contents.debug = int(self.params['debug'])

    def __del__(self):
        if self.tag_detector_ptr is not None:
            # destroy the tag families
            for family, tf in self.tag_families.items():
                if 'tag16h5' == family:
                    self.libc.tag16h5_destroy.restype = None
                    self.libc.tag16h5_destroy(tf)
                elif 'tag25h9' == family:
                    self.libc.tag25h9_destroy.restype = None
                    self.libc.tag25h9_destroy(tf)
                elif 'tag36h11' == family:
                    self.libc.tag36h11_destroy.restype = None
                    self.libc.tag36h11_destroy(tf)
                elif 'tagCircle21h7' == family:
                    self.libc.tagCircle21h7_destroy.restype = None
                    self.libc.tagCircle21h7_destroy(tf)
                elif 'tagCircle49h12' == family:
                    self.libc.tagCircle49h12_destroy.restype = None
                    self.libc.tagCircle49h12_destroy(tf)
                elif 'tagCustom48h12' == family:
                    self.libc.tagCustom48h12_destroy.restype = None
                    self.libc.tagCustom48h12_destroy(tf)
                elif 'tagStandard41h12' == family:
                    self.libc.tagStandard41h12_destroy.restype = None
                    self.libc.tagStandard41h12_destroy(tf)
                elif 'tagStandard52h13' == family:
                    self.libc.tagStandard52h13_destroy.restype = None
                    self.libc.tagStandard52h13_destroy(tf)

            # destroy the detector
            self.libc.apriltag_detector_destroy.restype = None
            self.libc.apriltag_detector_destroy(self.tag_detector_ptr)

    def detect(self, img, estimate_tag_pose=False, camera_params=None):
        """Run detectons on the provided image. The image must be a grayscale
           image of type np.uint8.
        # TODO get rid of the magic numbers
        """

        assert len(img.shape) == 2
        assert img.dtype == np.uint8

        c_img = self._convert_image(img)

        return_info = []

        # detect apriltags in the image
        self.libc.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)
        detections = self.libc.apriltag_detector_detect(
            self.tag_detector_ptr, c_img)

        apriltag = ctypes.POINTER(_ApriltagDetection)()

        for i in range(0, detections.contents.size):

            # extract the data for each apriltag that was identified
            zarray_get(detections, i, ctypes.byref(apriltag))

            tag = apriltag.contents

            # np.zeros((3,3))  # Don't ask questions, move on with your life
            homography = _matd_get_array(tag.H).copy()
            center = np.ctypeslib.as_array(tag.c, shape=(2,)).copy()
            corners = np.ctypeslib.as_array(tag.p, shape=(4, 2)).copy()

            detection = Detection()
            detection.tag_family = ctypes.string_at(tag.family.contents.name)
            detection.id = tag.id
            detection.hamming = tag.hamming
            detection.decision_margin = tag.decision_margin
            detection.homography = homography
            detection.center = center
            detection.corners = corners

            if estimate_tag_pose:
                # TODO better to ask for forgiveness than permission
                if camera_params is None:
                    raise ValueError(
                        "camera_params must be provided to detect if estimate_tag_pose is set to True")
                tag_size = MARKER.by_id(tag.id).size

                camera_fx, camera_fy, camera_cx, camera_cy = [
                    c for c in camera_params]

                info = _ApriltagDetectionInfo(det=apriltag,
                                              tagsize=tag_size,
                                              fx=camera_fx,
                                              fy=camera_fy,
                                              cx=camera_cx,
                                              cy=camera_cy)
                pose = _ApriltagPose()

                self.libc.estimate_tag_pose.restype = ctypes.c_double
                err = self.libc.estimate_tag_pose(
                    ctypes.byref(info), ctypes.byref(pose))

                detection.pose_R = _matd_get_array(pose.R).copy()
                detection.pose_T = _matd_get_array(pose.t).copy()
                detection.pose_err = err

                detection.dist = np.linalg.norm(detection.pose_T)

                rotations = transform.Rotation.from_matrix(detection.pose_R)
                rotations = rotations.as_euler("xyz", degrees=True)
                detection.rotation = Coords(*rotations)

                # Find the rotation about an axis by using the arctan2 of the
                # two other vectors perpendicular to that axis
                # TODO make clearer and only compute x, y
                bearings = (float(np.degrees(np.arctan2(u, v)))
                            for u, v in itertools.combinations(detection.pose_T, 2))
                detection.bearing = Coords(*bearings)

            # Append this dict to the tag data array
            return_info.append(detection)

        self.libc.image_u8_destroy.restype = None
        self.libc.image_u8_destroy(c_img)

        self.libc.apriltag_detections_destroy.restype = None
        self.libc.apriltag_detections_destroy(detections)

        return return_info

    def _convert_image(self, img):

        height = img.shape[0]
        width = img.shape[1]

        self.libc.image_u8_create.restype = ctypes.POINTER(_ImageU8)
        c_img = self.libc.image_u8_create(width, height)

        tmp = _image_u8_get_array(c_img)

        # copy the opencv image into the destination array, accounting for the
        # difference between stride & width.
        tmp[:, :width] = img

        # tmp goes out of scope here but we don't care because
        # the underlying data is still in c_img.
        return c_img
