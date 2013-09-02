import cv2
import numpy as np


class ImageProcessing:
    def __init__(self, area_threshold=4000):
        self.area_threshold = area_threshold
        self.ras_MIN = np.array([150, 80, 80], np.uint8)
        self.ras_MAX = np.array([175, 255, 255], np.uint8)

    def recognize_marker(self, frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, self.ras_MIN, self.ras_MAX)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.area_threshold > max_area:
                max_area = area
                best_cnt = cnt
        if max_area > self.area_threshold:
            M = cv2.moments(best_cnt)
            cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            return (cx, cy)
        else:
            return (None, None)
