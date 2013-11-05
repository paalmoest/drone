import pickle
from pykalman import KalmanFilter, UnscentedKalmanFilter
import numpy as np
import pylab as pl


class Main():

    def __init__(self):
        self.altitude = []
        self.co = []
        self.state = [0, 0]
        self.covariance = np.eye(1)
        test = 'test_16'
        self.baro = pickle.load(
            open('data/older/althold/%s/barometer.dump' % test))
        self.sonar = pickle.load(
            open('data/older/althold/%s/sonar.dump' % test))
        #transition_covariance = np.eye(2)
        random_state = np.random.RandomState(0)
        observation_covariance = np.eye(1) * 2
        transition_covariance = np.eye(2) * 0.001
        random_state = np.random.RandomState(0)

        self.kf = UnscentedKalmanFilter(
            self.transition_function, self.observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            random_state = np.random.RandomState(0),
        )

    def transition_function(self, state, noise):
        dt = 0.10
        a = state[0] + state[1] * dt + np.sin(noise[0])
        try:
            b = state[1] + noise[1]
        except:
            b = state[1] + noise[0]
        return np.array([a, b])
        # return a

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0],
        ])
        return np.dot(C, state) + noise

    def update_filter(self, value):
        dt = 0.10
        if abs(value[0] - self.state[0]) > 3:
            value = None
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                value,
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
# main.learn()

main.draw_fig()
