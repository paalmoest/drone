import pickle
from pykalman import KalmanFilter
import numpy as np
import pylab as pl

#kf = KalmanFilter(em_vars=['transition_matrices', 'observation_matrices', 'transition_covariance', 'observation_covariance'])


class Main():

    def __init__(self):
        self._observations = pickle.load(open('marker_observations.dump'))
#        self.observations = [[marker.x, mark for marker in self._observations if marker]]
        #transition_covariance = np.eye(4) * 0.00294393
        transition_covariance = np.eye(4) * 0.0001
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
            observation_covariance=np.eye(4) * 1,  # Q
        )
        self.states = []
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        self.x = []
        self.y = []

    def update(self, t, value):
        dt = 0.10
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                value,
                transition_matrix=np.array([
                                           [1, 0, 1, 0],
                                           [0, 1, 0, 1],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]]),
                observation_matrix=np.array([
                                            [1, 0, 0, 0],
                                            [0, 0, 1, 0],
                                            ]),
                # observation_offset = np.array([, 0, 0])
                # observation_covariance=np.array(0.1*np.eye(1))
            )
        )
        self.states.append(self.state[0])

    def update_filter(self, value):
        dt = 1
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
                                           [0, 0, 0, 1]
                                           ]),
                observation_matrix=np.array([
                                            [1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 0],
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
                self.update_filter([marker.x, (240 - marker.y), 1, 1])
            else:
                self.update_filter(None)

    def draw_fig(self):
        pl.figure(dpi=80)
        pl.xlim(-500, 500)
        pl.ylim(-500, 500)
               # obs_scatter = pl.scatter(x, y, marker='x', color='b',
            #             label='observations')
        xline = [0, 320, 320, 0, 0]
        yline = [0, 0, 240, 240, 0]
        position_line = pl.plot(self.x[0:355], self.y[0:355],
                                linestyle='-', marker='o', color='r',
                                label='position est.')
        position_line = pl.plot(xline, yline,
                                linestyle='-', marker='o', color='g',
                                label='FOV')

        #lines_sonar = pl.plot(self.sonar, color='b')
       # lines_filt = pl.plot(self.states, color='r')
       # pl.legend((lines_true[0]), ('true'))
        pl.show()


main = Main()
main.run2()
main.draw_fig()
