import cv2
import numpy as np


class ImageProcessing:
    def __init__(self, **kwargs):
        self.area_threshold = kwargs.get('area_threshold', 1000)
        #self.color_min = np.array([150, 80, 80], np.uint8)
        #self.color_max = np.array([175, 255, 255], np.uint8)
        self.color_min = np.array([0, 80, 80], np.uint8)
        self.color_max = np.array([10, 255, 255], np.uint8)

    def recognize_marker(self, frame):
        frame = cv2.blur(frame, (3, 3))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, self.color_min, self.color_max)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.area_threshold > max_area:
                max_area = area
                best_cnt = cnt
        if max_area > self.area_threshold:
            approx = cv2.approxPolyDP(best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            if len(approx) == 4:
                M = cv2.moments(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                return (cx, cy, best_cnt)
            else:
                return (None, None, None)
        else:
            return (None, None, None)


    def recognize_marker2(self, frame):
        frame = cv2.blur(frame, (3, 3))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, self.color_min, self.color_max)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.area_threshold > max_area:
                max_area = area
                best_cnt = cnt
        if max_area > self.area_threshold:
            #approx = cv2.approxPolyDP(best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            #if len(approx) == 4:
            M = cv2.moments(best_cnt)
            cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            return (cx, cy, best_cnt)
        else:
            return (None, None, None)

    def draw(self, frame, cx, cy, best_cnt):
        x, y, w, h = cv2.boundingRect(best_cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, 255, -1)
        cv2.putText(frame, 'X %s ' % cx, (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        cv2.putText(frame, 'Y %s ' % cy, (20, 60), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
