from pykalman import AdditiveUnscentedKalmanFilter
import numpy as np
import time


class UKFPosition():
    def __init__(self, autopilot):
        self.state = [0, 0]
        self.covariance = np.eye(2)
        observation_covariance = np.eye(1) * 0.5
        transition_covariance = np.eye(2) * 0.001
        self.autopilot = autopilot
        self.dt = 0.02
        self.previous_update = None
        self.kf = AdditiveUnscentedKalmanFilter(
            self.additive_transition_function,
            self.additive_observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            random_state=np.random.RandomState(0),

        )

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        if not self.previous_update:
            self.previous_update = time.time()

        self.dt = time.time() - self.previous_update
        a = (state[0] + (state[1] * self.dt)) + noise[0]
        c1 = - 0.1
        c2 = 1
        b = c1 * (c2 * ((np.cos(self.autopilot.heading) * np.sin(self.autopilot.angle_x) * np.cos(self.autopilot.angle_y)) - ((np.sin(self.autopilot.heading) * np.sin(self.autopilot.angle_y))))) + noise[1]
        self.previous_update = time.time()
        return np.array([a, b])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0],
        ])
        return np.dot(C, state) + noise

    def additive_transition_function(self, state):
        return self.transition_function(state, np.array([0, 0]))

    def additive_observation_function(self, state):
        return self.observation_function(state, np.array([0, 0]))

    def update_filter(self):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                [self.autopilot.x_distance_to_marker],
            )
        )