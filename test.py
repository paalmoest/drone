import cv2
import numpy as np #   break
import sys

cap = cv2.VideoCapture(0)
#cam_width = 320
#cam_height = 240
#cap.set(3, cam_width)
#cap.set(4, cam_height)
ras_MIN = np.array([150, 80, 80], np.uint8)
ras_MAX = np.array([175, 255, 255], np.uint8)
i = 0
while True:
    flag, frame = cap.read()
    if i % 5 == 0:
    #smooth image
        frame = cv2.blur(frame, (3, 3))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, ras_MIN, ras_MAX)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
           # print "area", max_area
            if area > max_area:
                max_area = area
                best_cnt = cnt
        if max_area:
            M = cv2.moments(best_cnt)
            cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            cv2.circle(frame, (cx, cy), 5, 255, -1)
        destRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        sys.stdout.write(frame.tostring())
        cv2.waitKey(5)
    #if cv2.waitKey(5)==27:
    #   break
i += 1
cv2.destroyAllWindows()
