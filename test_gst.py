#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import sys


#gst-launch v4l2src ! video/x-raw-yuv, width=320, height=240 ! ffmpegcolorspace ! jpegenc ! multipartmux ! queue !  tcpserversink port=5001


class Main:
    def __init__(self):
    	self.mainloop = gobject.MainLoop()

        self.pipeline = gst.Pipeline("pipeline")

        self.videosrc = gst.element_factory_make("v4l2src", "v4l2")

        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string('video/x-raw-yuv, width=640, height=480'))

        self.colorspace = gst.element_factory_make("ffmpegcolorspace", "colorspace")
        self.enc = gst.element_factory_make("jpegenc", "enc")
	#self.enc = gst.element_factory_make("dspmp4venc", "enc")
	#self.enc = gst.element_factory_make("dspjpegenc", "enc")

        self.mux = gst.element_factory_make("multipartmux", "mux")

        self.queue = gst.element_factory_make("queue", "buffer")

        self.tcpsink = gst.element_factory_make("tcpserversink", "sink")
        self.tcpsink.set_property("host", "129.241.103.121")
#	self.tcpsink.set_property("host", "10.0.0.1")
        self.tcpsink.set_property("port", 5001)

        self.pipeline.add_many(self.videosrc, self.vfilter, self.colorspace, self.enc, self.mux, self.queue, self.tcpsink)
        gst.element_link_many(self.videosrc, self.vfilter, self.colorspace, self.enc, self.mux, self.queue, self.tcpsink)

        self.pipeline.set_state(gst.STATE_PLAYING)

start = Main()
start.mainloop.run()
