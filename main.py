#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np
from image_processing import ImageProcessing
from autopilot import AutoPilot


class Main:
    def __init__(self, width=432, height=240, h264=False, device='/dev/device0', host='127.0.0.1', port='5000'):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")

        self.image_processing = ImageProcessing()
        #self.autopilot = AutoPilot()

        self.state = None
        self.width = width
        self.height = height
        if h264:
            self.videosrc = gst.parse_launch('uvch264_src device=/dev/video1 name=src auto-start=true src.vfsrc')
        else:
            self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')

        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string('image/jpeg, width=%s, height=%s, framerate=30/1' % (str(self.width), str(self.height))))
        self.queue = gst.element_factory_make("queue", "queue")

        self.udpsink = gst.element_factory_make('udpsink', 'udpsink')
        self.rtpjpegpay = gst.element_factory_make('rtpjpegpay', 'rtpjpegpay')
        self.udpsink.set_property('host', '127.0.0.1')
        self.udpsink.set_property('port', 5000)

        self.pipeline.add_many(self.videosrc, self.queue, self.vfilter, self.rtpjpegpay, self.udpsink)
        gst.element_link_many(self.videosrc, self.queue, self.vfilter, self.rtpjpegpay, self.udpsink)

        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)  # Sending frames to onVideBuffer where openCV can do processing.

        self.pipeline.set_state(gst.STATE_PLAYING)
        self.i = 0

    def init_videofeed(self):
        pass


    def onVideoBuffer(self, pad, idata):
        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
        self.i += 1
        if self.i % 5 == 0:
            cx, cy, best_cnt = self.image_processing.recognize_marker(frame)
            print cx
            print cy
        return True

start = Main(640, 480)
#start = Main(640,480)
start.mainloop.run()
