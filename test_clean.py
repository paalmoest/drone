import cv2
import cv
import numpy as np #   break
import sys
import time

cap = cv2.VideoCapture(0)
fps = 30
cam_width = 320
cam_height = 240

cap.set(3, cam_width)
cap.set(4, cam_height)
cap.set(cv.CV_CAP_PROP_FOURCC, cv.CV_FOURCC('M', 'J', 'P', 'G'))
while True:
    flag, frame = cap.read()
    #im_rgb = cv2.CreateImage(self.size,  opencv.IPL_DEPTH_8U, 3)
    #v2.CvtColor(frame, im_rgb, cv2.CV_BGR2RGB)
    destRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.waitKey(5)
    #time.sleep(int(1.0/fps))
   #sys.stdout.write(destRGB.tostring())
cv2.destroyAllWindows()