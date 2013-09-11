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
        cv2.waitKey(5)

        pad = next(self.queue.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)  # Sending frames to onVideBuffer where openCV can do processing.
        self.pipeline.set_state(gst.STATE_PLAYING)
        self.i = 0
        self.roll = 0
        self.pitch = 0
        self.mainloop.run()

    def generate_filename(self, testname):
        return testname + '_' + datetime.now().strftime('%m%B%Y_%H_%M_%S') + '.avi'

    def push_to_pipeline(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        buff = gst.Buffer(np.getbuffer(frame_rgb.astype("uint8")))
        self.appsrc.emit("push-buffer", buff)

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
            cx, cy, best_cnt = self.image_processing.recognize_marker2(frame)
            self.roll, self.pitch = self.autopilot.simulate_position_hold(cx, cy)
            if cx:
                M = cv2.moments(best_cnt)
                x, y, w, h = cv2.boundingRect(best_cnt)
                cv2.circle(frame, (cx, cy), 5, 255, -1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                #cv2.rectangle(img, , (, color[, thickness[, lineType[, shift]]]
        cv2.putText(frame, 'roll %s ' % self.roll, (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        cv2.putText(frame, 'pitch %s ' % self.pitch, (20, 60), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        cv2.imshow('drone eye', frame)

        return True
