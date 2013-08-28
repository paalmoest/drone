#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import sys
import cv2
import numpy as np
import scipy
#from numpy import array, getbuffer, frombuffer


#gst-launch v4l2src ! video/x-raw-yuv, width=320, height=240 ! ffmpegcolorspace ! jpegenc ! multipartmux ! queue !  tcpserversink port=5001
#gst-dsp-parse v4l2src device=/dev/video0 always-copy=FALSE ! 'video/x-raw-yuv,width=320,height=240, framerate=15/1' ! dsph264enc ! rtph264pay ! udpsink host=129.241.103.209 port=1234


class Main:
    def __init__(self):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.width = 640
        self.height = 480
        self.h264_width = 1280
        self.h264_height = 720
        #self.videosrcpyt = gst.element_factory_make("v4l2src", "v4l2")
        #self.videosrc = gst.element_factory_make("videotestsrc", "v4l2")
        self.videosrc = gst.parse_launch('uvch264_src device=/dev/video1 name=src auto-start=true src.vfsrc')
        self.fakesink = gst.parse_launch('fakesink src.vidsrc')
        self.queue3 = gst.element_factory_make("queue", "buffer")
        self.vfilter3 = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter3.set_property('caps', gst.caps_from_string('video/x-h264,width=%s,height=%s,framerate=30/1,profile=constrained-baseline' % (str(self.h264_width), str(self.h264_height))))
        #! xvimagesink src.vidsrc ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1,profile=constrained-baseline ! h264parse ! mp4mux ! filesink location=test.mp4
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string('video/x-raw-rgb,format=BGR3, width=%s, height=%s,framerate=30/1' % (str(self.width), str(self.height))))
       #self.vfilter.set_property('caps', gst.caps_from_string('video/x-raw-rgb,format=RGB3, width=640, height=480'))
        self.vfilter2 = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter2.set_property('caps', gst.caps_from_string('video/x-raw-rgb,format=RGB3, width=640, height=480'))


        self.queue = gst.element_factory_make("queue", "queue")
        self.appsrc = gst.element_factory_make("appsrc", "mysrc")

        self.colorspace = gst.element_factory_make("ffmpegcolorspace", "colorspace")
        self.mux = gst.element_factory_make("avimux", "mux")

        self.h264parse = gst.element_factory_make("h264parse", "h264parse")
        self.mp4mux = gst.element_factory_make("mp4mux", "mp4mux")
        self.rtph264pay = gst.element_factory_make("rtph264pay", "rtph264pay")
        self.queue = gst.element_factory_make("queue", "buffer")
        self.queue2 = gst.element_factory_make("queue", "buffer")
        self.udpsink = gst.element_factory_make('udpsink','udpsink')
        self.udpsink.set_property('host', '10.0.0.100')
        self.udpsink.set_propert('port', '5001')
        self.ras_MIN = np.array([150, 80, 80], np.uint8)
        self.ras_MAX = np.array([175, 255, 255], np.uint8)

   #     self.tcpsink = gst.element_factory_make("tcpserversink", "sink")
        #self.tcpsink.set_property("host", "129.241.103.121")
    #	self.tcpsink.set_property("host", "10.0.0.1")
     #   self.tcpsink.set_property("port", 5001)
        self.enc = gst.element_factory_make("jpegenc", "enc")
     #   self.fakesink = gst.element_factory_make('fakesink', 'fake')
        self.colorspace = gst.element_factory_make("ffmpegcolorspace", "colorspace")
        self.colorspace2 = gst.element_factory_make("ffmpegcolorspace", "colorspace")
        self.sink = gst.element_factory_make('xvimagesink', 'xvimagesink')
        #self.sink = gst.element_factory_make('xvimagesink', 'xvimagesink')
        self.xvimagesink = gst.element_factory_make('xvimagesink', 'xvimagesink')
        self.filesink = gst.element_factory_make('filesink')
        #self.appsink = gst.element_factory_make('filesink')
        self.tcpsink = gst.element_factory_make("tcpserversink", "sink")
        self.tcpsink.set_property("port", 5001)
        self.i = 0
       # self.filesink.set_property('location', '/home/paalmm/ntnu/master/drone/from_buffer.avi')
        self.fakesink2 = gst.element_factory_make('fakesink', 'fake')
        self.pipeline2 = gst.Pipeline("pipeline")
        self.videoparse = gst.element_factory_make('videoparse', 'videoparse')
        self.videoparse.set_property('width', self.width)
        self.videoparse.set_property('height', self.height)
        self.videoparse.set_property('format', 14)
        self.videoparse.set_property('framerate', gst.Fraction(30/1))
        #self.pipeline2.add_many(self.appsrc, self.vfilter2, self.colorspace, self.sink)
      #  gst.element_link_many(self.appsrc, self.vfilter2, self.colorspace, self.sink)

        self.pipeline2.add_many(self.appsrc, self.videoparse, self.colorspace, self.enc, self.tcpsink)
        gst.element_link_many(self.appsrc, self.videoparse, self.colorspace, self.enc, self.tcpsink)


        self.pipeline.add_many(self.videosrc, self.queue, self.vfilter, self.fakesink, self.queue3, self.vfilter3, self.h264parse, self.rtph264pay,self.udpsink)
        gst.element_link_many(self.videosrc, self.queue, self.vfilter, self.fakesink, self.queue3, self.vfilter3, self.h264parse, self.rtph264pay,self.udpsink )
        #self.appsrc.props.emit_signals = True

        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)

     #   pad2 = next(self.fakesink2.sink_pads())
       # pad2.add_buffer_probe(self.onVideoBuffer2)
        #self.appsrc.connect('need-data', self.need_data)

        self.pipeline.set_state(gst.STATE_PLAYING)
        self.pipeline2.set_state(gst.STATE_PLAYING)

    def need_data(self, src, length):
      #  print "hello"
      #  print len(buff)
        if self.buff:
            print "im here"
            self.appsrc.emit("push-buffer", self.buff)
        return True

    def onVideoBuffer2(self, pad, idata):
      #  print len(idata)
       # print "onvideobuffer 2"
        image2 = np.ndarray(
            shape=(480, 640, 3),
            dtype=np.uint8,
            buffer=idata,
        )
       # print image2.shape()
       #int len(image2)
        scipy.misc.imsave('outfile2.jpg', image2)
        #exit()
       # print image2
        return True
    def onVideoBuffer(self, pad, idata):
        #print "oki"
        #print 'idata ', type(idata), len(idata)
        #print len(idata)

        image = np.ndarray(
            shape=(self.height, self.width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        frame = image
        #buff = gst.Buffer(image.tostring())
        #print type(buff)
        #print len(buff)
       # print image
       # print image
       # frame = cv2.blur(image, (3, 3))
        #global globvar
        self.i += 1
        frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if self.i % 1 == 0:
            hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            #hsv_img = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(hsv_img, self.ras_MIN, self.ras_MAX)
            thresh2 = thresh.copy()
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area
                    best_cnt = cnt
            if max_area > 10000:
                M = cv2.moments(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, 255, -1)
                #print "hello"
            cv2.putText(frame, 'OMG THIS WORKS !', (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        #scipy.misc.imsave('outfile3.jpg', frame)
        #self.buff = gst.Buffer(np.getbuffer(image.astype("uint8")))
        #print type(self.buff)

        #self.appsrc.emit("push-buffer", self.buff)
        #buff = idata
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        buff = gst.Buffer(np.getbuffer(frame_rgb.astype("uint8")))
       # print type(buff)
        #print i
        #print len(buff)
        #print len(buff)
        self.appsrc.emit("push-buffer", buff)
    #        self.appsrc.
      #  self.appsrc.emit("push-buffer", buff)
        #print frame
        #self.push_buffer(idata)
        #hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #cv2.imshow('hello', frame
        #hsv_img = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)
        return True
start = Main()
start.mainloop.run()
