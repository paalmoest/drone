#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np
import time
from position_controller import PositionController
from state_estimation import StateEstimationAltitude
from image_processing import ImageProcessing
from autopilot import AutoPilot
#v4l2-ctl --list-formats-ext


class Main:

    def __init__(self, **kwargs):
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
        self.image_processing = ImageProcessing(area_threshold=10)
       # self.state_estimation = StateEstimationAltitude()
        #self.position_controller = PositionController(self.state_estimation)
        self.autopilot = AutoPilot(
            self.state_estimate,
            self.position_controller)
        if h264:
            self.videosrc = gst.parse_launch(
                'uvch264_src device=/dev/video0 name=src auto-start=true src.vfsrc')
        else:
            self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        fps = 30
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string(
            'image/jpeg, width=%s, height=%s, framerate=30/1' % (str(cam_width), str(cam_height))))
        self.queue = gst.element_factory_make("queue", "queue")

        self.udpsink = gst.element_factory_make('udpsink', 'udpsink')
        self.rtpjpegpay = gst.element_factory_make('rtpjpegpay', 'rtpjpegpay')
        self.udpsink.set_property('host', host)
        self.udpsink.set_property('port', port)

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
        self.pipeline.set_state(gst.STATE_PLAYING)
        self.i = 0
        gobject.threads_init()
        context = self.mainloop.get_context()
        previous_update = time.time()
        fpstime = time.time()
        while True:
            try:
                context.iteration(False)
                if time.time() >= previous_update:
                    self.autopilot._read_sensors()
                    if self.autopilot.auto_switch > 1700:
                        self.position_controller.holdAltitude()
                        self.position_controller.headingHold()
                        self.autopilot.send_control_commands()
                        print self.autopilot.print_commands()
                    previous_update = time.time() + 0.20
            except KeyboardInterrupt:
                fps = self.i / (time.time() - fpstime)
                print 'fps %f ' % fps
                self.autopilot.dump_log()

    def onVideoBuffer(self, pad, idata):
        try:
            image = np.asarray(
                bytearray(idata),
                dtype=np.uint8,
            )
            frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
            self.i += 1
            marker = self.image_processing.recognize_marker(frame)
            self.autopilot.update_marker(marker)
            return True
        except:
            return True
