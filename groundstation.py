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

        gobject.threads_init()
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")

        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.port = 5000

        self.buildPipeline()
        fpstime = time.time()

        context = self.mainloop.get_context()
        self.pipeline.set_state(gst.STATE_PLAYING)

        self.i = 0
        self.j = 0
        while True:
            time.sleep(0.5)
            #self.j += 1
            # print self.j
           # self.autopilot.read_sensors()
            context.iteration(False)

    def onVideoBufferRaw(self, pad, idata):
        # print "hello"

        hsv_min = np.array([100, 130, 130], np.uint8)
        hsv_max = np.array([120, 255, 255], np.uint8)
        image = np.ndarray(
            shape=(self.cam_height, self.cam_width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        self.i += 1
       # cv2.putText(image, 'OMG THIS WORKS !', (
       #     20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))


        return True

    def buildPipeline(self):
        self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        self.videosrc = gst.element_factory_make('udpsrc', 'udprc')
        self.videosrc.set_property('port', self.port)
        self.videosrc.set_property('caps', gst.caps_from_string(
            'application/x-rtp, media=(string)video, clock-rate=(int)90000, format=15,  width=(string)320, height=(string)240, sampling=(string)RGB'))

        self.queue = gst.element_factory_make("queue", "queue")
        self.rtpdepayraw = gst.element_factory_make(
            'rtpvrawdepay', 'rtpvrawdepay')

        self.udpsink = gst.element_factory_make('udpsink', 'udpsink')
        self.udpsink.set_property('host', '127.0.0.1')
        self.udpsink.set_property('port', 5000)
        self.colorspace = gst.element_factory_make(
            "ffmpegcolorspace", "ffmpegcolorspace")
        self.videoparse = gst.element_factory_make("videoparse", "colorspace")
        self.videoparse.set_property('format', 14)
        self.xvimagesink = gst.element_factory_make(
            'xvimagesink', 'xvimagesink')
        self.xvimagesink.set_property('sync', False)
        self.pipeline.add_many(
            self.videosrc, self.rtpdepayraw, self.videoparse, self.queue, self.colorspace, self.xvimagesink)
        gst.element_link_many(
            self.videosrc, self.rtpdepayraw, self.videoparse,  self.queue, self.colorspace, self.xvimagesink)
        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBufferRaw)
#


loop = Main()
