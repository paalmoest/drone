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
     #   process_noise = 1
        transition_covariance = np.array([
                                         [0.000025, 0, 0.0005, 0],
                                         [0, 0.000025, 0, 0.0005],
                                         [0.0005, 0, 0.001, 0],
                                         [0, 0.000025, 0, 0.001]
                                         ])
        i = 0
        for o in self._observations:
            if o:
                print 'x: %d Y: %d, inde: %d ' % (o.x, o.y, i)
            else:
                print None
            i += 1

        #self.observation_covariance = np.eye(1) * 0.00140264
        self.observation_covariance = np.eye(2)
 # s       self.n_timesteps = len(self.observations)
        self.kf = KalmanFilter(
            transition_covariance=transition_covariance,  # H
            observation_covariance=np.eye(2) * 1,  # Q
        )
        self.states = []
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        self.x = []
        self.y = []

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
        # print value
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                value,
                transition_matrix=np.array([
                                           [1, 0, dt, 0],
                                           [0, 1, 0, dt],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]]),
                observation_matrix=np.array([
                                            [1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            ]),
            )
        )
        # print '%d, %d ' % (self.state[0], self.state[2])
        self.x.append(self.state[0])
        self.y.append(self.state[1])
        # sprint self.state[2]
        # print self.state[2]

    def run(self):
        for t in range(self.n_timesteps - 1):
            self.update(t, self.observations[t + 1])
            #self.update_ukf(t, self.baro[t + 1])

    def run2(self):
        for marker in self._observations:
            if marker:
                self.update_filter([marker.x, (240 - marker.y)])
            else:
                self.update_filter(None)

    def draw_fig(self):
        pl.figure(dpi=80)
        pl.xlim(-500, 500)
        pl.ylim(-50, 500)
               # obs_scatter = pl.scatter(x, y, marker='x', color='b',
            #             label='observations')
        xline = [0, 320, 320, 0, 0]
        yline = [0, 0, 240, 240, 0]
        
        position_line = pl.plot(self.x[0:355], self.y[0:355],
                                linestyle='-', marker='o', color='r',
                                label='position est.')
       
        FOV = pl.plot(xline, yline,
                      linestyle='-', marker='o', color='g',
                                label='FOV')
        pl.show()
"""
        learning = pl.plot(self.lx[0:355], self.ly[0:355],
                           linestyle='-', marker='o', color='r',
                           label='position est.')
"""
        #lines_sonar = pl.plot(self.sonar, color='b')
       # lines_filt = pl.plot(self.states, color='r')
       # pl.legend((lines_true[0]), ('true'))
        


main = Main()
main.run2()
#main.learn()

main.draw_fig()
