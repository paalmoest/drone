#!/usr/bin/python

import pygst
pygst.require("0.10")
import gst
import gobject
import cv2
import numpy as np


class Main:

    def __init__(self):
        self.mainloop = gobject.MainLoop()
        self.pipeline = gst.Pipeline("pipeline")
        self.width = 320
        self.height = 240
        self.videosrc = gst.element_factory_make('v4l2src', 'v4l2src')
        self.videosrc.set_property('device', '/dev/video1')
        self.vfilter = gst.element_factory_make("capsfilter", "vfilter")
        self.vfilter.set_property('caps', gst.caps_from_string('video/x-raw-rgb,format=RGB3, width=%s, height=%s,framerate=30/1' % (str(320), str(240))))
      #  self.vfilter.set_property('caps', gst.caps_from_string(
       #     'image/jpeg, width=%s, height=%s, framerate=30/1' % (str(self.width), str(self.height))))
        self.colorspace = gst.element_factory_make("colorspace", "colorspace")
        self.queue = gst.element_factory_make("queue", "queue")
        self.fakesink = gst.element_factory_make('fakesink', 'fake')
        self.pipeline.add_many(
            self.videosrc, self.vfilter,self.fakesink)
        gst.element_link_many(self.videosrc, self.vfilter, self.fakesink)
        self.i = 0
        pad = next(self.fakesink.sink_pads())
        pad.add_buffer_probe(self.onVideoBuffer)
        self.pipeline.set_state(gst.STATE_PLAYING)



    def onVideoBuffer(self, pad, idata):
        #hsv_min = np.array([105, 120, 120], np.uint8)
        #hsv_max = np.array([120, 255, 255], np.uint8)
        #hsv_min = np.array([0, 120, 120], np.uint8)
        #hsv_max = np.array([15, 255, 255], np.uint8)
        hsv_min = np.array([110, 130, 130], np.uint8)
        hsv_max = np.array([120, 255, 255], np.uint8)
        image = np.ndarray(
            shape=(self.height, self.width, 3),
            dtype=np.uint8,
            buffer=idata,
        )
        #hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        thresh = cv2.inRange(hsv_img, hsv_min, hsv_max)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        frame  = image
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
        print max_area
        if max_area > 100:
            approx = cv2.approxPolyDP(
                best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
          #  print len(approx)
            if len(approx) == 4:
                M = cv2.moments(best_cnt)
               # cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                x, y, w, h = cv2.boundingRect(best_cnt)
                areal = w * h
                rect = cv2.minAreaRect(best_cnt)
                #print 'center1: %d %d ' % (cx, cy)
                #print 'center2: %d %d' % (rect[0][0], rect[0][1])
               # print 'angle rangel : %d' % rect[2]
                #box = cv2.cv.BoxPoints(rect)
                #box = np.int0(box)
                # print box
                # cv2.drawContours(frame,[box],0,(0,0,255),2)
                #cv2.circle(frame, (cx, cy), 5, 255, -1)
                cv2.circle(frame, (int(rect[0][0]), int(rect[0][1])), 5, 200, -1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
               
        cv2.imshow('drone eye', thresh2)
        #cv2.waitKey(10)
        #frame = image
        self.i += 1
      #  print self.i
       # frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return True


    # if cv2.waitKey(5)==27:
    #   break
start = Main()
start.mainloop.run()
cv2.destroyAllWindows()
