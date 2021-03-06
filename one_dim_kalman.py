import pickle
from pykalman import KalmanFilter
import numpy as np
import pylab as pl


class Main():

    def __init__(self):
        self._observations = pickle.load(open('dumps/marker_observations.dump'))
        transition_covariance = np.array([
                                         [0.025, 0.005],
                                         [0.0005, 0.01],
                                         ])
        self.kf = KalmanFilter(
            transition_covariance=transition_covariance,  # H
            observation_covariance=np.eye(1) * 1,  # Q
        )
        self.altitude = []
        self.co = []
        self.state = [0, 0]
        self.covariance = np.eye(2)
        test = 'test_9'
        self._baro = pickle.load(open('data/duedalen/%s/barometer.dump' % test))
        self._throttle = pickle.load(open('data/duedalen/%s/throttle.dump' % test))
        self._sonar = pickle.load(open('data/12.11.13/%s/sonar.dump' % test))
        self
        self.acceleration = pickle.load(open('data/12.11.13/%s/acceleration.dump' % test))
        self.z_velocity = [a.z_velocity for a in self.acceleration]
        self.baro = [i[1] for i in self._baro]
        self.throttle = [(i[1] - 1000.0 ) / (1000.0) for i in self._throttle]
        print self.throttle
        self.sonar = [i[1] for i in self._sonar]
        self.cam_alt = [marker.z if marker else np.ma.masked for marker in self._observations]
        for i in xrange(len(self._baro) - 1):
            dt = self._baro[i + 1][0] - self._baro[i][0]
            print dt
       # self.zvelo = [i.value for i in self._zvelo]
       # print self.zvelo
       #exit()
    def learn(self):
        dt = 0.10
        kf = KalmanFilter(
            em_vars=['transition_covariance', 'observation_covariance'],
            observation_covariance=np.eye(2) * 1,
            transition_covariance=np.array([
                                           [0.000025, 0, 0.0005, 0],
                                           [0, 0.000025, 0, 0.0005],
                                           [0.0005, 0, 0.001, 0],
                                           [0, 0.000025, 0, 0.001]
                                           ]),
            transition_matrices=np.array([
                                         [1, 0, dt, 0],
                                         [0, 1, 0, dt],
                                         [0, 0, 1, 0],
                                         [0, 0, 0, 1]
                                         ]),
            observation_matrices=np.array([
                                          [1, 0, 0, 0],
                                          [0, 1, 0, 0],
                                          ]),
        )
        states, co = kf.em(self.offline_data).smooth(self.offline_data)
        print kf.transition_covariance
        self.lx = [s[0] for s in states]
        self.ly = [s[1] for s in states]

        # print kf.transition_covariance
        # print kf.observation_covariance
    def update_filter(self, value, control):
        dt = 0.10
        #if abs(value[0] - self.state[0]) > 10:
        #    value = Non
        control = control / 200
        print value

        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                value,
                transition_matrix=np.array([
                                           [1, dt],
                                           [0, 1]
                                           ]),
                observation_matrix=np.array([
                                            [1, 0]
                                            ]),
                transition_offset=np.array([control])
            )
        )
        self.co.append(self.covariance[0])
        self.altitude.append(self.state[0])

    def run(self):
        self.i = 0
        for c in self.throttle:
            self.update_filter([self.cam_alt[self.i]], c)
            self.i += 1

    def draw_fig(self):
        pl.figure(dpi=80)
        pl.plot(self.altitude, color='b')
        pl.plot(self.cam_alt, color='r')
        pl.plot(self.baro, color='g')
        pl.show()

        #pl.figure(dpi=80)
        #pl.plot(self.z_velocity, color='g')
        #pl.show()
       # lines_filt = pl.plot(self.states, color='r')
       # pl.legend((lines_true[0]), ('true'))


main = Main()
main.run()
#main.learn()

main.draw_fig()
