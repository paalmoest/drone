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
import os
#v4l2-ctl --list-formats-ext


class Main:

    def __init__(self, **kwargs):

        gobject.threads_init()
        self.mainloop = gobject.MainLoop()
     #   self.pipeline = gst.Pipeline("pipeline")

        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.port = 5000

        # self.buildPipeline()
        #self.buildPipeLineFile()
        self.parsePipeline()
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

        hsv_min = np.array([100, 120, 120], np.uint8)
        hsv_max = np.array([120, 255, 255], np.uint8)
        image = np.ndarray(
            shape=(self.cam_height, self.cam_width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        self.i += 1
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, hsv_min, hsv_max)
       # image = thresh.copy()
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
                best_cnt = cv2.convexHull(best_cnt)
        print max_area
        if max_area > 300:
            approx = cv2.approxPolyDP(
                best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            if len(approx) == 4:
                M = cv2.moments(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                x, y, w, h = cv2.boundingRect(best_cnt)
                #areal = w * h
                rect = cv2.minAreaRect(best_cnt)

               # box = cv2.cv.BoxPoints(rect)
            #    box = np.int0(box)
              #  print box
               # cv2.drawContours(image, box, 0,(0,0,255),2)
                # cv2.imWrite()
              #  cv2.circle(image, (int(rect[0][0]), int(rect[0][1])), 5, 200, -1)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
            #    _image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                # cv2.putText(image, 'OMG THIS WORKS !', (
       #     20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))

        return True

    def get_test_number(self, mypath, number):
        tmp = mypath + str(number)
        if not os.path.isfile('data/video/%s' % tmp):
            return tmp
        else:
            return self.get_test_number(mypath, number + 1)

    def parsePipeline(self):
        self.pipeline = gst.parse_launch("udpsrc port=5000 name=udp  ! rtpvrawdepay ! videoparse format=14 ! queue name=pad  ! ffmpegcolorspace ! tee name=my_videosink ! queue!  avimux ! filesink name=my_filesink my_videosink. ! queue !   xvimagesink sync=false")
        self.udpsrc = self.pipeline.get_by_name('udp')
        self.udpsrc.set_property('caps', gst.caps_from_string(
            'application/x-rtp, media=(string)video, clock-rate=(int)90000, format=15,  width=(string)320, height=(string)240, sampling=(string)RGB'))
        self.pad_element = self.pipeline.get_by_name('pad')
        pad = next(self.pad_element.sink_pads())
        pad.add_buffer_probe(self.onVideoBufferRaw)
        myfile = 'test_'
        number = 1
        myfile = self.get_test_number(myfile, number)
        self.filesink = self.pipeline.get_by_name('my_filesink')
        self.filesink.set_property('location', 'data/video/%s.avi' % myfile)



    def buildPipeLineFile(self):
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
        self.tee = gst.element_factory_make('tee', 'my_videosink')
        self.tee.set_property('name', 'my_videosink')
        self.jpegenc = gst.element_factory_make('jpegenc', 'jpegenc')
        self.mux = gst.element_factory_make('avimux', 'avimux')
        self.queue2 = gst.element_factory_make('queue', 'queue2')
        self.queue3 = gst.element_factory_make('queue', 'queue3')
        # tee name=my_videosink ! jpegenc ! avimux ! filesink
        # location=video.avi my_videosink. ! queue ! xvimagesink
        self.filesink = gst.parse_launch('filesink my_videosink.')
        self.filesink.set_property('location', 'lol.avi')
        self.pipeline.add_many(
            self.videosrc, self.rtpdepayraw, self.videoparse, self.queue, self.colorspace, self.tee, self.queue2, self.mux, self.filesink, self.queue3, self.xvimagesink)
        gst.element_link_many(
            self.videosrc, self.rtpdepayraw, self.videoparse,  self.queue, self.colorspace, self.tee, self.queue2, self.mux, self.filesink, self.queue3, self.xvimagesink)
        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBufferRaw)

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
