from pykalman import AdditiveUnscentedKalmanFilter
import numpy as np
import time


class UKFPosition():

    def __init__(self):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        observation_covariance = np.eye(2) * 0.1
        transition_covariance = np.eye(4) * 0.01
        self.init_yaw = 0
        self.previous_update = None
        self.kf = AdditiveUnscentedKalmanFilter(
            self.additive_transition_function,
            self.additive_observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
        )

    def transition_function(self, state, noise):
        if not self.previous_update:
            self.previous_update = time.time()

        x = (state[0] + (state[1] * self.dt)) + noise[0]
        x_velocity = state[1] + noise[1]
        y = (state[2] + (state[3] * self.dt)) + noise[2]
        y_velocity = state[3] + noise[3]

        self.previous_update = time.time()
        return np.array([x, x_velocity, y, y_velocity])

    def observation_function(self, state, noise):
        C = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ])
        return np.dot(C, state) + noise

    def additive_transition_function(self, state):
        return self.transition_function(state, np.array([0, 0, 0, 0]))

    def additive_observation_function(self, state):
        return self.observation_function(state, np.array([0, 0]))

    def update_filter(self, observations):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
            )
        )

    def calculateHorizontalVelocity(self, roll, pitch, yaw):
        #yaw = yaw - self.init_yaw
        yaw = 0
        c1 = 7
        x_v = c1 * np.cos(yaw) * np.sin(roll) * np.cos(
            pitch) - np.sin(yaw) * np.sin(pitch)
        y_v = c1 * -np.sin(yaw) * np.sin(roll) * np.cos(
            pitch) - np.cos(yaw) * np.sin(pitch)
        return x_v, y_v