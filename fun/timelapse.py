import cv2
import time


capture = cv2.VideoCapture(0)
img_num = 0
while True:
	flag, frame = capture.read()
	destRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	img_num += 1 
	cv2.imwrite('%s.jpeg' % img_num, frame)
	time.sleep(2)


