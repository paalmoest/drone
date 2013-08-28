import cv2
import cv
import numpy as np #   break
import sys
import time

cap = cv2.VideoCapture("/home/paal/ntnu/master/drone/crash.MP4")

while True:
    flag, frame = cap.read()
cv2.destroyAllWindows()
