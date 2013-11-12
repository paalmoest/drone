import cv2
import cv
from image_processing import ImageProcessing
import pickle
import pylab as pl
import time


class OfflineVideo():

    def __init__(self, video_file="data/test_5/drone_eye.avi"):
        self.cap = cv2.VideoCapture(video_file)
        self.cap.set(cv.CV_CAP_PROP_FPS, 30)
        self.image_processing = ImageProcessing(area_treshold=300)
        self.writer = cv2.VideoWriter(filename="kalman_tracking5.avi", fps=30, frameSize=(
            320, 240), fourcc=cv.CV_FOURCC('M', 'J', 'P', 'G'))
        self.cam_altitude = []
        self.observations = []
        #self.autopilot = AutoPilot(simulate=True, thrust_step=30, pixel_threshold=10, cam_width=320, cam_height=240)
        # self.ras_MIN = np.array([150, 80, 80], np.uint8)
       # self.ras_MAX = np.array([175, 255, 255], np.uint8)

    def run(self):
        i = 0
        try:
            while True:
                flag, frame = self.cap.read()
                if i % 3 == 0:
                    marker = self.image_processing.recognize_marker(frame)
                    if marker:
                        self.cam_altitude.append(marker.z)
                        self.observations.append(marker)
                        cv2.circle(frame, (marker.x, marker.y), 5, 255, -1)
                       # print marker.rect
                        # print rect[]
                        x, y, w, h = cv2.boundingRect(marker.best_cnt)
                        cv2.rectangle(
                            frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(frame, 'altitude %s ' % marker.z, (
                            20, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))
                        cv2.putText(frame, 'X: %s Y: %s ' %
                                    (str(marker.x), str(marker.y)),
                                     (20, 50),
                                     cv2.FONT_HERSHEY_PLAIN,
                                     2,
                                     (0, 255, 0))
                    else:
                        self.cam_altitude.append(None)
                        self.observations.append(None)
                i += 1
                #time.sleep(0.3)
                self.writer.write(frame)
                cv2.imshow('drone eye', frame)
                cv2.waitKey(5)
        except:
            # draw estimates
            pickle.dump(self.cam_altitude, open('cam_alt.dump', 'wb'))
            pickle.dump(
                self.observations,
                open('marker_observations5.dump', 'wb'))
            # pl.figure(size=(320,240))
            x = [o.x for o in self.observations if o]
            y = [o.y for o in self.observations if o]
            # obs_scatter = pl.scatter(x, y, marker='x', color='b',
            #             label='observations')

            position_line = pl.plot(x, y,
                                    linestyle='-', marker='o', color='r',
                                    label='position est.')
            #lines_true = pl.plot(self.cam_altitude, color='b')
           # observations = pl.plot(x, color='b')
            # lines_filt = pl.plot(filtered_state_means, color='r')
            # pl.legend((lines_true[0]), ('Camera altitude'))
            pl.show()


offline_video = OfflineVideo()
offline_video.run()
