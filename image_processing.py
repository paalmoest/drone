import cv2
import numpy as np
import cv


class ImageProcessing:
    def __init__(self, area_threshold=100):
        self.area_threshold = area_threshold
        self.ras_MIN = np.array([150, 80, 80], np.uint8)
        self.ras_MAX = np.array([175, 255, 255], np.uint8)

    def recognize_marker(self, frame):
        frame = cv2.blur(frame, (3, 3))
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
            return (cx, cy, best_cnt)
        else:
            return (None, None, None)

    def draw(self, frame, cx, cy, best_cnt):
        x, y, w, h = cv2.boundingRect(best_cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, 255, -1)
        cv2.putText(frame, 'X %s ' % cx, (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        cv2.putText(frame, 'Y %s ' % cy, (20, 60), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))