#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np
from image_processing import ImageProcessing


class Main:
    def __init__(self, width=432, height=240, port='5000'):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.recording = False
        self.width = width
        self.height = height
        self.image_processing = ImageProcessing()
        self.udpsrc = gst.element_factory_make("udpsrc")
        self.udpsrc.set_property('port', 5000)
        self.udpsrc.set_property("caps", gst.caps_from_string('application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)jpeg'))

        #gst-launch -v udpsrc port=5000 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)jpeg' !  rtpjpegdepay ! jpegdec ! xvimagesink sync=false

        self.queue = gst.element_factory_make("queue", "queue")
        self.jpegdec = gst.element_factory_make("jpegdec", "jpegdec")
        self.jpegdec2 = gst.element_factory_make("jpegdec", "jpegdec2")

        self.rtpjpegpay = gst.element_factory_make('rtpjpegdepay', 'rtpjpegdepay')
        #self.filesink = gst.element_factory_make('filesink', 'filesink')
        #self.filesink.set_property('location', '/home/paal/ntnu/master/drone/KAFFE.avi')
       # self.udpsink.set_property('host', '127.0.0.1')
       # self.udpsink.set_property('port', 5000)

        if self.recording:
            self.filesink = gst.element_factory_make('filesink', 'filesink')
            self.filesink.set_property('location', self.filename)
            self.mux = gst.element_factory_make('avimux', 'avimux')
            self.pipeline.add_many(self.udpsrc, self.rtpjpegpay, self.queue, self.jpegdec, self.mux, self.filesink)
            gst.element_link_many(self.udpsrc, self.rtpjpegpay, self.queue,  self.jpegdec, self.mux, self.filesink)
        else:
            self.fakesink = gst.element_factory_make('fakesink', 'fake')
            self.pipeline.add_many(self.udpsrc, self.rtpjpegpay, self.queue, self.jpegdec, self.fakesink)
            gst.element_link_many(self.udpsrc, self.rtpjpegpay, self.queue,  self.jpegdec, self.fakesink)

        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)  # Sending frames to onVideBuffer where openCV can do processing.

        self.pipeline.set_state(gst.STATE_PLAYING)
        self.i = 0
        cv2.waitKey(3)

    def onVideoBuffer(self, pad, idata):
        image = np.asarray(
            bytearray(idata),
            dtype=np.uint8,
        )
        frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
       	cv2.cvtColor(frame, cv2.COLOR_YCR_CB2BGR)
        #self.i += 1
        #print self.i
        if self.i % 5 == 0:
            cx, cy, best_cnt = self.image_processing.recognize_marker(frame)
            if cx:
                #print len(approx)
                #if len(approx) == 4:
                    #print "its a square"
                #elif len(approx) > 15:
                 #   print "its a circle!"
                M = cv2.moments(best_cnt)
               # approx = cv2.approxPolyDP(best_cnt, 0.01 * cv2.arcLength(best_cnt, True), True)
                x, y, w, h = cv2.boundingRect(best_cnt)
                cv2.circle(frame, (cx, cy), 5, 255, -1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                #cv2.rectangle(img, , (, color[, thickness[, lineType[, shift]]]
        cv2.imshow('drone eye', frame)
       # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        #buff = gst.Buffer(np.getbuffer(frame_rgb.astype("uint8")))
       # self.appsrc.emit("push-buffer", buff)

        return True

start = Main()
#start = Main(640, 480)
start.mainloop.run()
