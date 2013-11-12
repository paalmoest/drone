import pickle
from pykalman import KalmanFilter
import numpy as np
import pylab as pl


class Main():

    def __init__(self):
        self._observations = pickle.load(open('marker_observations5.dump'))
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
        test = 'test_8'
        self._baro = pickle.load(open('data/12.11.13/%s/barometer.dump' % test))
        self._sonar = pickle.load(open('data/12.11.13/%s/sonar.dump' % test))
        self.acceleration = pickle.load(open('data/12.11.13/%s/acceleration.dump' % test))
        self.z_velocity = [a.z_velocity for a in self.acceleration]
        for i in self.acceleration:
            print i.z_velocity
        self.baro = [i[1] for i in self._baro]
        self.sonar = [i[1] for i in self._sonar]
        self.cam_alt = [marker.z if marker else np.ma.masked for marker in self._observations]
     
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
    def update_filter(self, value):
        dt = 0.10
        #if abs(value[0] - self.state[0]) > 10:
        #    value = None
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
            )
        )
        self.co.append(self.covariance[0])
        self.altitude.append(self.state[0])

    def run(self):
        for o in self.baro:
            self.update_filter([o])

    def draw_fig(self):
        pl.figure(dpi=80)
        pl.plot(self.sonar, color='b')
        pl.plot(self.altitude, color='g')
        pl.show()

        pl.figure(dpi=80)
        pl.plot(self.z_velocity, color='g')
        pl.show()
       # lines_filt = pl.plot(self.states, color='r')
       # pl.legend((lines_true[0]), ('true'))


main = Main()
main.run()
#main.learn()

main.draw_fig()
