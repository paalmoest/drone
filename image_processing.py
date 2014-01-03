import cv2
import numpy as np
import time
#f-length 320x240  = 308
#f-length 160x90 = 121.742


class Marker():
    def __init__(self, **kwargs):
        self.x = kwargs.get('cx', None)
        self.y = kwargs.get('cy', None)
        self.best_cnt = kwargs.get('best_cnt', None)
        d = kwargs.get('d', None)
        if d:
            self.z = self.calculate_alitude(d)
        self.timestamp = time.time()

    def get_altitude(self):
        if self.z:
            return self.z
        else:
            return None

    def calculate_alitude(self, d):
        Z = 0.31 * (121.742 / d)
        return Z


class ImageProcessing:
    def __init__(self, **kwargs):
        self.area_threshold = 200
        #self.hsv_min = np.array([150, 80, 80], np.uint8)
        #self.hsv_max = np.array([180, 255, 255], np.uint8)
        #self.hsv_min = np.array([100, 80, 80], np.uint8)
        #self.hsv_max = np.array([120, 255, 255], np.uint8)
        #self.hsv_min = np.array([80, 130, 130], np.uint8)
        #self.hsv_max = np.array([150, 255, 255], np.uint8)
        self.hsv_min = np.array([110, 130, 130], np.uint8)
        self.hsv_max = np.array([120, 255, 255], np.uint8)

    def recognize_marker(self, frame):
        frame = cv2.blur(frame, (3, 3))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        thresh = cv2.inRange(hsv_img, self.hsv_min, self.hsv_max)
        #thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
        if max_area > self.area_threshold:
            approx = cv2.approxPolyDP(best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            if len(approx) == 4:
                M = cv2.moments(best_cnt)
                rect = cv2.minAreaRect(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                print 'X: %d Y: %d Area: %f lenght: %f altitude: %f' % (cx, cy, max_area, rect[1][0])
                return Marker(cx=cx, cy=cy, d=rect[1][0], best_cnt=best_cnt)
            else:
                print 'X: None Y: None  Area: %f' % max_area
                return None
        else:
            print 'X: None Y: None  Area: %f' % max_area
            return None
