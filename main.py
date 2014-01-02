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
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.verbose = kwargs.get('verbose', True)
        self.debug = kwargs.get('debug', False)
        self.cam_width = kwargs.get('cam_width', 640)
        self.cam_height = kwargs.get('cam_height', 480)
        self.host = kwargs.get('host', '127.0.0.1')
        self.port = kwargs.get('port', 5000)
        h264 = kwargs.get('h264', False)
        heading_pid = kwargs.get('heading_pid', None)
        altitude_pid = kwargs.get('altitude_pid', None)
        roll_pid = kwargs.get('roll_pid', None)
        #heading_d = kwargs.get('heading_p', 0)
        self.marker_spotted = False
        self.image_processing = ImageProcessing()
        self.state_estimate = StateEstimationAltitudeSonar()
        self.state_estimate_marker = StateEstimationMarkerOnline()
        self.autopilot = AutoPilot(
            self.state_estimate, self.state_estimate_marker)
        self.ukf_position = UKFPosition2(self.autopilot)

        self.position_controller = PositionController(
            self.autopilot, self.state_estimate, self.state_estimate_marker, roll_pid=roll_pid, heading_pid=heading_pid, altitude_pid=altitude_pid)
        if h264:
            self.videosrc = gst.parse_launch(
                'uvch264_src device=/dev/video0 name=src auto-start=true src.vfsrc')
        else:
            self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        #self.buildJPEGVideofeed()
        self.buildRawVideofeed()

        self.pipeline.set_state(gst.STATE_PLAYING)
        self.i = 0
        gobject.threads_init()
        context = self.mainloop.get_context()
        fpstime = time.time()
        TenHZtask = time.time()
        while True:
            try:
                context.iteration(False)
                self.autopilot.read_sensors()
                if time.time() >= TenHZtask:
                    self.autopilot.calcualteMarkerDistance()
                    try:
                        pass
                        # self.ukf_position.update_filter()
                        # self.ukf_position.update_filter()
                    except np.linalg.linalg.LinAlgError:
                        print "omg"
                    print self.print_ukf4d()
                    self.autopilot.log_ukf(self.ukf_position.state)
                    TenHZtask = time.time() + 0.1
                if self.autopilot.auto_switch > 1500:
                    self.position_controller.altitudeHoldSonarKalman()
                    self.autopilot.send_control_commands()
                else:
                   # print self.print_attiude()
                    self.position_controller.reset_targets()

            except KeyboardInterrupt:
                fps = self.i / (time.time() - fpstime)
                print 'fps %f ' % fps
                self.autopilot.dump_log()
                self.autopilot.disconnect_from_drone()

    def onVideoBuffer(self, pad, idata):

        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
        self.i += 1
        marker = self.image_processing.recognize_marker(frame)
        self.autopilot.update_marker(marker)
        return True

    def onVideoBufferRaw(self, pad, idata):
        image = np.ndarray(
            shape=(self.cam_height, self.cam_width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        self.i += 1
        marker = self.image_processing.recognize_marker(image)
        self.autopilot.update_marker(marker)
        return True

    def print_ukf_test(self):
        return 'x: %.5f y: %.5f roll: %.5f pitch: %.5f yaw: %.5f' % (
            self.ukf_position.state[0],
            self.ukf_position.state[2],
            self.ukf_position.state[4],
            self.ukf_position.state[5],
            self.ukf_position.state[6],
        )

    def print_ukf3d(self):
        return 'roll: %.5f pitch: %.5f yaw: %.5f' % (
            self.ukf_position.state[0],
            self.ukf_position.state[1],
            self.ukf_position.state[2],
        )

    def print_ukf2d(self):
        return 'x: %.5f speed: %.5f ' % (
            self.ukf_position.state[0],
            self.ukf_position.state[1],
        )

    def print_ukf4d(self):
        return 'x: %.4f x_speed: %.4f y: %.4f y_speed: %.4f angle_x: %.2f, angle_y: %.2f' % (
            self.ukf_position.state[0],
            self.ukf_position.state[1],
            self.ukf_position.state[2],
            self.ukf_position.state[3],
            self.autopilot.angle_x,
            self.autopilot.angle_y,
        )

    def print_attiude(self):
        return 'x: %.2f y: %.2f' % (
            self.autopilot.angle_x,
            self.autopilot.angle_y,

        )

    def buildRawVideofeed(self):
        self.vfilter.set_property('caps', gst.caps_from_string(
            'video/x-raw-rgb,format=RGB3, width=%s, height=%s,framerate=30/1' % (str(self.cam_width), str(self.cam_height))))
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
