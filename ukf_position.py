import pickle
from pykalman import KalmanFilter, UnscentedKalmanFilter
import numpy as np
import pylab as pl


class Main():

    def __init__(self):
        self.estiamted_alt = []
        self.co = []
        self.state = [0, 0]
        self.covariance = np.eye(2)
        observation_covariance = np.eye(1) * 0.5
        transition_covariance = np.eye(2) * 0.001
        self.angle_x = [i.roll for i in self.attitude]
        self.angle_y = [i.pitch for i in self.attitude]
        self.angle_z = [i.yaw for i in self.attitude]
        self.baro = [a.barometer for a in self.altitude]

        self.kf = UnscentedKalmanFilter(
            self.transition_function, self.observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            random_state=np.random.RandomState(0),

        )

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        a = (state[0] + (state[1] *  self.dt)) * noise[0]
        c1 = 1
        c2 = 1
        b = c1 * (c2 * ((np.cos(self.autopilot.heading) * np.sin(self.autopilot.angle_x) * np.cos(self.autopilot.angle_y)) - ((np.sin(self.autopilot.heading) * np.sin(self.autopilot.angle_y))))) + noise[1]
        return np.array([a, b])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0],
        ])
        return np.dot(C, state) + noise

    def update_filter(self, value):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                value,
            )
        )
