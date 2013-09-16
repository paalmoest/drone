import cv2
import cv
import numpy as np
from image_processing import ImageProcessing


class OfflineVideo():
    def __init__(self, video_file=None):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv.CV_CAP_PROP_FPS, 30)
        self.image_processing = ImageProcessing(area_threasho)
        # self.ras_MIN = np.array([150, 80, 80], np.uint8)
       # self.ras_MAX = np.array([175, 255, 255], np.uint8)

    def run(self):
        i = 0
        while True:
            flag, frame = self.cap.read()
            if i % 1 == 0:
                cx, cy, best_cnt = self.image_processing.recognize_marker(frame)
                if cx:
                  #  frame = self.draw(frame, cx, cy, best_cnt)
                    approx = cv2.approxPolyDP(best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
                    if len(approx) == 4:
                        frame = self.draw(frame, cx, cy, best_cnt)
            i += 1
            cv2.imshow('drone eye', frame)
            cv2.waitKey(10)

    def draw(self, frame, cx, cy, best_cnt):
        x, y, w, h = cv2.boundingRect(best_cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, 255, -1)
        cv2.putText(frame, 'X %s ' % cx, (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        cv2.putText(frame, 'Y %s ' % cy, (20, 60), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
        return frame

offline_video = OfflineVideo()
offline_video.run()
