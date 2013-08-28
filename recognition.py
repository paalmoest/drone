import cv2
import cv
import numpy as np

cap = cv2.VideoCapture(1)
cam_width = 640
cam_height = 480

cap.set(3, cam_width)
cap.set(4, cam_height)
ras_MIN = np.array([150, 80, 80], np.uint8)
ras_MAX = np.array([175, 255, 255], np.uint8)
range_min = np.array([110, 80, 0], np.uint8)
range_max = np.array([120, 255, 255], np.uint8)

#
#range_min = np.array([110, 100, 80], np.uint8)
#range_max = np.array([125, 255, 255], np.uint8)


i = 0
fps = 30
#fourcc = cv.CV_FOURCC('I', '4', '2', '0')
#fourcc = cv.CV_FOURCC('M', 'J', 'P', 'G')
#writer = cv2.VideoWriter('out.avi', fourcc, fps, (cam_width, cam_height), 1)
center = [cam_width / 2, cam_height / 2]
butter_zone_x = [center[0] - 50, center[0] + 50]
butter_zone_y = [center[1] - 50, center[1] + 50]
cx = "n/a"
cy = "n/a"

#print butter_zone_x[0]
#print butter_zone_x[1]
#exit()
def navigate(cx, xy):
    if butter_zone_x[0] <= cx <= butter_zone_x[1]:
        print "XX HOLD X  position! "
    else:
            if cx < center[0]:
                print "Roll LEFT <--"
            else:
                print "ROLL RIGHT ->>"
    if butter_zone_y[0] <= cy <= butter_zone_y[1]:
        print "YY HOLD Y  position! "
    else:
        if cy < center[1]:
            print "pitch forward !"
        else:
            print "pitch Back !"

while True:
    flag, frame = cap.read()
    #smooth image
    if i % 1 == 0:
        frame = cv2.blur(frame, (3, 3))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, range_min, range_max)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
           # print "area", max_area
            if area > 100:
                cv2.drawContours(frame, cnt, -1,(0, 255, 0 ),3 )
                approx = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt,True),True)
                approx2 = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt,True),True)
                if len(approx) == 4:
                #if area > max_area:
                    max_area = area
                    best_cnt = cnt         
        if max_area > 1000:
           # cv2.drawContours(frame, cnt, -1,(0, 255, 0 ),3 )
         #   approx = cv2.approxPolyDP(best_cnt, 0.01 * cv2.arcLength(cnt,True),True)
          #  if len(approx) == 4:
           #     print "its a square !"
            #elif len(approx) > 15:
             #   print "its a circle"
            #approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            #print len(approx)
              #  print "its a square"
            #elif len(approx) > 15:
            #    print "its a circle!"
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                M = cv2.moments(best_cnt)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, 255, -1)
              #  navigate(cx, cy)

    cv2.putText(frame, 'X %s Y: %s ' % (str(cx), str(cy)), (20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255 , 0))
    i += 1
    cv2.imshow('drone eye', frame)
    cv2.waitKey(5)
    #destRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #writer.write(frame)
        #sys.stdout.write(frame.tostring())
    #if cv2.waitKey(5)==27:
    #   break
        #
cv2.destroyAllWindows()
