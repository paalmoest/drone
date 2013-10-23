import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np
from datetime import datetime


class Groundstation:
    def __init__(self, autopilot, image_processing, **kwargs):
        self.autopilot = autopilot
        self.image_processing = image_processing
        port = kwargs.get('port', 5000)

        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.record = kwargs.get('record', False)
        self.udpsrc = gst.element_factory_make("udpsrc")
        self.udpsrc.set_property('port', port)
        self.udpsrc.set_property("caps", gst.caps_from_string('application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)jpeg'))
        self.queue = gst.element_factory_make("queue", "queue")
        self.jpegdec = gst.element_factory_make("jpegdec", "jpegdec")
        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.rtpjpegpay = gst.element_factory_make('rtpjpegdepay', 'rtpjpegdepay')
        if self.record:
            self.filesink = gst.element_factory_make('filesink', 'filesink')
            testname = kwargs.get('testname', 'copter_test')
            self.filesink.set_property('location', self.generate_filename(testname))
            self.mux = gst.element_factory_make('avimux', 'avimux')
            self.pipeline.add_many(self.udpsrc, self.rtpjpegpay, self.queue, self.jpegdec, self.mux, self.filesink)
            gst.element_link_many(self.udpsrc, self.rtpjpegpay, self.queue, self.jpegdec, self.mux, self.filesink)
        else:
            self.fakesink = gst.element_factory_make('fakesink', 'fake')
            self.pipeline.add_many(self.udpsrc, self.rtpjpegpay, self.queue, self.jpegdec, self.fakesink)
            gst.element_link_many(self.udpsrc, self.rtpjpegpay, self.queue,  self.jpegdec, self.fakesink)
        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)  # Sending frames to onVideBuffer where openCV can do processing.
        self.pipeline.set_state(gst.STATE_PLAYING)
        self.init_appsrc()
        self.i = 0
        self.roll = 0
        self.pitch = 0
        self.mainloop.run()

    def generate_filename(self, testname):
        return testname + '_' + datetime.now().strftime('%m%B%Y_%H_%M_%S') + '.avi'

    def init_appsrc(self):
        self.appsrc = gst.element_factory_make("appsrc", "mysrc")
        self.pipeline2 = gst.Pipeline("pipeline")
        self.colorspace = gst.element_factory_make("ffmpegcolorspace", "colorspace")
        self.videoparse = gst.element_factory_make('videoparse', 'videoparse')
        self.videoparse.set_property('width', self.cam_width)
        self.videoparse.set_property('height', self.cam_height)
        self.videoparse.set_property('format', 14)
        self.tcpsink = gst.element_factory_make("tcpserversink", "sink")
        self.tcpsink.set_property("port", 5001)
        self.enc = gst.element_factory_make("jpegenc", "enc")
        self.videoparse.set_property('framerate', gst.Fraction(30/1))
        self.pipeline2.add_many(self.appsrc, self.videoparse, self.enc, self.tcpsink)
        gst.element_link_many(self.appsrc, self.videoparse, self.enc, self.tcpsink)
        self.pipeline2.set_state(gst.STATE_PLAYING)

    def onVideoBuffer(self, pad, idata):
        try:
            image = np.asarray(
                bytearray(idata),
                dtype=np.uint8,
            )
            frame = cv2.imdecode(image, cv2.CV_LOAD_IMAGE_UNCHANGED)
            self.i += 1
            if self.i % 1 == 0:
                cx, cy, best_cnt = self.image_processing.recognize_marker(frame)
                self.roll, self.pitch = self.autopilot.simulate_position_hold(cx, cy)
                if cx:
                    x, y, w, h = cv2.boundingRect(best_cnt)
                    cv2.circle(frame, (cx, cy), 5, 255, -1)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, 'roll %s ' % self.roll, (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
            cv2.putText(frame, 'pitch %s ' % self.pitch, (20, 60), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
            buff = gst.Buffer(np.getbuffer(frame.astype("uint8")))
            self.pipeline2 = gst.Pipeline("pipeline")
            self.appsrc.emit("push-buffer", buff)
            return True
        except:
            return True
