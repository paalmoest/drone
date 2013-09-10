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
    def __init__(self, width=432, height=240, h264=False, device='/dev/video0', host='127.0.0.1', port='5000'):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")

        self.image_processing = ImageProcessing()
        self.autopilot = AutoPilot(thrust_step=30, pixel_threshold=50,time_interval=0.1,cam_width=width, cam_height=height)
        
        self.width = width
        self.height = height
        self.cx = 0
        if h264:
            self.videosrc = gst.parse_launch('uvch264_src device=/dev/video1 name=src auto-start=true src.vfsrc')
        else:
            self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')

        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string('image/jpeg, width=%s, height=%s, framerate=30/1' % (str(self.width), str(self.height))))
        self.queue = gst.element_factory_make("queue", "queue")

        self.udpsink = gst.element_factory_make('udpsink', 'udpsink')
        self.rtpjpegpay = gst.element_factory_make('rtpjpegpay', 'rtpjpegpay')
        self.udpsink.set_property('host', host)
        self.udpsink.set_property('port', port)

        self.pipeline.add_many(self.videosrc, self.queue, self.vfilter, self.rtpjpegpay, self.udpsink)
        gst.element_link_many(self.videosrc, self.queue, self.vfilter, self.rtpjpegpay, self.udpsink)

        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)  # Sending frames to onVideBuffer where openCV can do processing.

        self.pipeline.set_state(gst.STATE_PLAYING)
        self.i = 0
        self.j = 0
        self.cx = None
        self.cy = None
        gobject.threads_init()
        context = self.mainloop.get_context()
        while True:
            context.iteration(False)
            if self.j % 3 == 0:
                sensor_data = self.autopilot.read_sensors()
                if sensor_data:
                    self.autopilot.set_state(sensor_data)
            #print self.autopilot.pp_receiver_commands()
            if self.j % 10 == 0:
                if self.cx and self.cy:
                    self.autopilot.position_hold(self.cx, self.cy)
            self.j += 1

    def onVideoBuffer(self, pad, idata):
        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
        self.i += 1
        if self.i % 5 == 0:
            self.cx, self.cy, best_cnt = self.image_processing.recognize_marker2(frame)
        return True

start = Main(640, 480)
#start = Main(640,480)
#start.mainloop.run()
