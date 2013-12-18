from pykalman import UnscentedKalmanFilter
import numpy as np
import time


class UKFPosition():
    def __init__(self, autopilot):
        self.state = [0, 0, 0]
        self.covariance = np.eye(3)
        observation_covariance = np.eye(1) * 0.5
        transition_covariance = np.eye(3) * 0.001
        self.autopilot = autopilot
        self.previous_update = None
        self.kf = UnscentedKalmanFilter(
            self.transition_function, self.observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            random_state=np.random.RandomState(0),

        )

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        if not self.previous_update:
            self.previous_update = time.time()

        self.dt = time.time() - self.previous_update
        a = state[0] + noise[0]
        b = state[1] + noise[1]
        c = state[2] + noise[2]
        return np.array([a, b, c])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ])
        return np.dot(C, state) + noise

    def update_filter(self):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                [self.autopilot.angle_x, self.autopilot.angle_y, self.autopilot.heading],
            )
        )
