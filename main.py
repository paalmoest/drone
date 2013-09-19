#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np


class Main:
    def __init__(self, autopilot, image_processing, **kwargs):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.verbose = kwargs.get('verbose', True)
        self.debug = kwargs.get('debug', False)
        cam_width = kwargs.get('cam_width', 640)
        cam_height = kwargs.get('cam_height', 480)
        host = kwargs.get('host', '127.0.0.1')
        port = kwargs.get('port', 5000)
        h264 = kwargs.get('h264', False)
        self.marker_spotted = False

        self.image_processing = image_processing
        self.autopilot = autopilot
        self.cx = 0
        if h264:
            self.videosrc = gst.parse_launch('uvch264_src device=/dev/video0 name=src auto-start=true src.vfsrc')
        else:
            self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')

        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string('image/jpeg, width=%s, height=%s, framerate=30/1' % (str(cam_width), str(cam_height))))
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
            try:
                context.iteration(False)
                if autopilot:
                    if self.j % 3 == 0:
                        self.autopilot.read_sensors()
                    if self.j % 10 == 0:
                        self.autopilot.test_roll(1530)  # roll right
                        #self.autopilot.test_pitch(1530) # pitch forward
                        #1self.autopilot.test_roll_pitch(1600, 1600)  # right,forward
                        if self.cx and self.cy:
                            #self.autopilot.position_hold(self.cx, self.cy)
                            self.marker_spotted = True
                        else:
                            self.marker_spotted = False
                        if self.verbose:
                            print self.autopilot.pp_receiver_commands() + " marker: " + str(self.marker_spotted)

                    self.j += 1
            except KeyboardInterrupt:
                self.autopilot.dump_log()



    def onVideoBuffer(self, pad, idata):
        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
        self.i += 1
        if self.i % 5 == 0:
            self.cx, self.cy, bounding_rectangle = self.image_processing.recognize_marker(frame)
        return True
