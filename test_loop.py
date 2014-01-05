#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np
import time
from position_controller import PositionController
from state_estimation import StateEstimationAltitudeSonar, StateEstimationMarkerOnline
from position_estimator import UKFPosition2
from image_processing import ImageProcessing
from autopilot import AutoPilot
#v4l2-ctl --list-formats-ext


class Main:

    def __init__(self, **kwargs):

        self.state_estimate = StateEstimationAltitudeSonar()
        self.autopilot = AutoPilot(self.state_estimate)

        gobject.threads_init()
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")

        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.host = kwargs.get('host', '127.0.0.1')
        self.port = kwargs.get('port', 5000)
        self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        self.videosrc.set_property('device', '/dev/video1')
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.image_processing = ImageProcessing()
        self.buildRawVideofeed()
        fpstime = time.time()

        context = self.mainloop.get_context()

        self.i = 0

        self.pipeline.set_state(gst.STATE_PLAYING)
        while True:
            print self.i
            context.iteration(True)

    def onVideoBuffer(self, pad, idata):

        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
        marker = self.image_processing.recognize_marker(frame)
       # self.autopilot.update_marker(marker)
        return True

    def onVideoBufferRaw(self, pad, idata):

        hsv_min = np.array([100, 130, 130], np.uint8)
        hsv_max = np.array([120, 255, 255], np.uint8)
        image = np.ndarray(
            shape=(self.cam_height, self.cam_width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        self.i += 1
        hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        thresh = cv2.inRange(hsv_img, hsv_min, hsv_max)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
        if max_area > 100:
            approx = cv2.approxPolyDP(
                best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            if len(approx) == 4:
                M = cv2.moments(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                x, y, w, h = cv2.boundingRect(best_cnt)
                areal = w * h
                rect = cv2.minAreaRect(best_cnt)
        #marker = self.image_processing.recognize_marker(image)
       # self.autopilot.update_marker(marker)
       	print self.i
        return True

    def buildRawVideofeed(self):

        self.vfilter.set_property('caps', gst.caps_from_string(
            'video/x-raw-rgb,format=RGB3, width=%d, height=%d,framerate=%s' % (self.cam_width, self.cam_height, '30/1')))
        self.queue = gst.element_factory_make("queue", "queue")
        self.fakesink = gst.element_factory_make('fakesink', 'fake')
        self.pipeline.add_many(
            self.videosrc, self.vfilter, self.fakesink)
        gst.element_link_many(
            self.videosrc, self.vfilter, self.fakesink)
        pad = next(self.fakesink.sink_pads())
        pad.add_buffer_probe(self.onVideoBufferRaw)

    def buildJPEGVideofeed(self):
        self.vfilter.set_property('caps', gst.caps_from_string(
            'image/jpeg, width=%s, height=%s, framerate=15/1' % (str(self.cam_width), str(self.cam_height))))
        self.queue = gst.element_factory_make("queue", "queue")
        self.udpsink = gst.element_factory_make('udpsink', 'udpsink')
        self.rtpjpegpay = gst.element_factory_make('rtpjpegpay', 'rtpjpegpay')
        self.udpsink.set_property('host', self.host)
        self.udpsink.set_property('port', self.port)
        self.pipeline.add_many(
            self.videosrc,
            self.queue,
            self.vfilter,
            self.rtpjpegpay,
            self.udpsink)
        gst.element_link_many(
            self.videosrc,
            self.queue,
            self.vfilter,
            self.rtpjpegpay,
            self.udpsink)
        pad = next(self.queue.sink_pads())
        # Sending frames to onVideBuffer where openCV can do processing.
        pad.add_buffer_probe(self.onVideoBuffer)


loop = Main()
