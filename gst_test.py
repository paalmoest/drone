#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np


class Main:
    def __init__(self):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.width = 320
        self.height = 240
        self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        #self.vfilter.set_property('caps', gst.caps_from_string(
        #    'video/x-raw-rgb,format=RGB3, width=%s, height=%s,framerate=30/1' % (str(self.width), str(self.height))))
      #  self.vfilter.set_property('caps', gst.caps_from_string(
      #      'image/jpeg, width=%s, height=%s, framerate=30/1' % (str(self.width), str(self.height))))

        self.queue = gst.element_factory_make("queue", "queue")
        self.fakesink = gst.element_factory_make('fakesink', 'fake')
        self.pipeline.add_many(
            self.videosrc, self.vfilter, self.fakesink)
        gst.element_link_many(self.videosrc, self.vfilter, self.fakesink)
        self.i = 0
        pad = next(self.fakesink.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)
        self.pipeline.set_state(gst.STATE_PLAYING)

        self.ras_MIN = np.array([150, 80, 80], np.uint8)
        self.ras_MAX = np.array([175, 255, 255], np.uint8)
    
    def onVideoBuffer(self, pad, idata):
        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        """
        image = np.ndarray(
            shape=(self.height, self.width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        """
        #frame = image
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
        self.i += 1
        print self.i
       # frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return True

start = Main()
start.mainloop.run()
