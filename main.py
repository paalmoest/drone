#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np
import time
from position_controller import PositionController
from state_estimation import StateEstimationAltitudeSonar
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
        self.host = kwargs.get('host', '10.0.0.44')
        self.port = kwargs.get('port', 5000)
        h264 = kwargs.get('h264', False)
        heading_pid = kwargs.get('heading_pid', None)
        altitude_pid = kwargs.get('altitude_pid', None)
        roll_pid = kwargs.get('roll_pid', None)
        pitch_pid = kwargs.get('pitch_pid', None)
        self.image_processing = ImageProcessing()
        self.state_estimate = StateEstimationAltitudeSonar()
        self.autopilot = AutoPilot(self.state_estimate, c1=kwargs.get('c1', 0))
        self.ukf_position = UKFPosition2(self.autopilot)
        self.position_controller = PositionController(
            self.autopilot, self.state_estimate, autoland_pid=kwargs.get('autoland_pid', None), roll_pid=roll_pid, pitch_pid=pitch_pid, heading_pid=heading_pid, altitude_pid=altitude_pid)
        if h264:
            self.videosrc = gst.parse_launch(
                'uvch264_src device=/dev/video0 name=src auto-start=true src.vfsrc')
        else:
            self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        # self.buildJPEGVideofeed()
        self.buildRawVideofeed()
        self.i = 0

        context = self.mainloop.get_context()
        previous_time = time.time()

        self.pipeline.set_state(gst.STATE_PLAYING)

        fpstime = time.time()
        while True:
            try:
                self.autopilot.read_sensors()
                if self.autopilot.auto_switch > 1700:
                    self.autopilot.update_linearKf()
                    self.position_controller.altitudeHoldSonarKalman()
                    self.position_controller.positionHold()
                    self.autopilot.send_control_commands()
                else:
                    print self.autopilot.print_commands()
                    self.position_controller.reset_targets()
            except KeyboardInterrupt:
                fps = self.i / (time.time() - fpstime)
                print 'fps %f ' % fps
                self.autopilot.dump_log()
                self.autopilot.disconnect_from_drone()
            context.iteration(False)

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
        print "hllllllllll"

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
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
               # best_cnt = cv2.convexHull(best_cnt)
        if max_area > 300:
            approx = cv2.approxPolyDP(
                best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            if len(approx) == 4:
                M = cv2.moments(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                #x, y, w, h = cv2.boundingRect(best_cnt)
                #areal = w * h
                #rect = cv2.minAreaRect(best_cnt)
                self.autopilot.cx = cx
                self.autopilot.cy = cy
                #self.autopilot.rect = rect
                self.autopilot.marker = True
                # print cx, cy
                #marker = self.image_processing.recognize_marker(image)
            else:
                self.autopilot.marker = False
       # self.autopilot.update_marker(marker)
        cv2.putText(image, '%.2f' % self.autopilot.altitude_sonar,
                    (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
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
            'video/x-raw-rgb,format=RGB3, width=%d, height=%d,framerate=%s' % (self.cam_width, self.cam_height, '20/1')))
        self.queue = gst.element_factory_make("queue", "queue")
        self.fakesink = gst.element_factory_make('fakesink', 'fake')
        self.rtpraw = gst.element_factory_make('rtpvrawpay', 'rtpvrawpay')
        self.udpsink = gst.element_factory_make('udpsink', 'udpsink')
        self.udpsink.set_property('host', self.host)
        self.udpsink.set_property('port', self.port)
        self.pipeline.add_many(
            self.videosrc, self.vfilter, self.queue, self.rtpraw, self.udpsink)
        gst.element_link_many(
            self.videosrc, self.vfilter, self.queue, self.rtpraw, self.udpsink)
        pad = next(self.queue.sink_pads())
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
