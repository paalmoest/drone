import pickle
from pykalman import KalmanFilter
import numpy as np
import pylab as pl


class Main():

    def __init__(self):
        self._observations = pickle.load(open('marker_observations.dump'))
        # transition_covariance = np.eye(4) * 0.00294393'
        self.offline_data = []
        for marker in self._observations:
            if marker:
                self.offline_data.append([marker.x, marker.y])
            else:
                pass
     #   process_noise = 10               x  Y      x
        transition_covariance = np.array([
                                         [0.000025, 0.0005],
                                         [0.0005, 0.001],
                                         ])
        self.kf = KalmanFilter(
            transition_covariance=transition_covariance,  # H
            observation_covariance=np.eye(1) * 1,  # Q
        )
        self.altitude = []
        self.co = []
        self.state = [0, 0]
        self.covariance = np.eye(2)
        test = 'test_16'
        self.baro = pickle.load(open('data/older/althold/%s/barometer.dump' % test))
        self.sonar = pickle.load(open('data/older/althold/%s/sonar.dump' % test))

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
        if abs(value[0] - self.state[0]) > 10:
            value = None
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
        for o in self.sonar:
            self.update_filter([o])

    def draw_fig(self):
        pl.figure(dpi=80)
        pl.plot(self.altitude, color='b')
        pl.plot(self.sonar, color='g')
        pl.show()

        pl.figure(dpi=80)
        pl.plot(self.co, color='g')
        pl.show()
       # lines_filt = pl.plot(self.states, color='r')
       # pl.legend((lines_true[0]), ('true'))


main = Main()
main.run()
#main.learn()

main.draw_fig()
