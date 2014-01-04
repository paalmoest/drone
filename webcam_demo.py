import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cam_width = 320
cam_height = 240

cap.set(3, cam_width)
cap.set(4, cam_height)
#ras_MIN = np.array([150, 80, 80], np.uint8)
#ras_MAX = np.array([175, 255, 255], np.uint8)
hsv_min = np.array([95, 80, 80], np.uint8)
hsv_max = np.array([115, 255, 255], np.uint8)
hsv_min = np.array([0, 150, 150], np.uint8)
hsv_max = np.array([20, 255, 255], np.uint8)
i = 0
while True:
    flag, frame = cap.read()
    if i % 1 == 0:
    # smooth image
        frame = cv2.blur(frame, (3, 3))
        #hsv_img = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv_img, hsv_min, hsv_max)
        thresh2 = thresh.copy()
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
        print max_area
        if max_area > 1000:
            approx = cv2.approxPolyDP(
                best_cnt, 0.1 * cv2.arcLength(best_cnt, True), True)
            print len(approx)
            if len(approx) == 4:
                #M = cv2.moments(best_cnt)
                #cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                
                x, y, w, h = cv2.boundingRect(best_cnt)
                areal = w * h
                rect = cv2.minAreaRect(best_cnt)
              #  print 'center1: %d %d ' % (cx, cy)
                print 'center2: %d %d' % (rect[0][0], rect[0][1])
                print 'angle rangel : %d' % rect[2]
                #box = cv2.cv.BoxPoints(rect)
                #box = np.int0(box)
                # print box
                # cv2.drawContours(frame,[box],0,(0,0,255),2)
                #cv2.circle(frame, (cx, cy), 5, 255, -1)
                cv2.circle(frame, (int(rect[0][0]), int(rect[0][1])), 5, 200, -1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # print "x : %s " % cx
        #cv2.imshow('drone eye', thre)
        #cv2.imshow('drone eye', frame)
        # sys.stdout.write(frame.tostring())
        cv2.waitKey(5)
    # if cv2.waitKey(5)==27:
    #   break
    i += 1
cv2.destroyAllWindows()
