from pykalman import AdditiveUnscentedKalmanFilter
import numpy as np
import time


class UKFPosition():

    def __init__(self, autopilot):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        observation_covariance = np.eye(2) * 0.1
        transition_covariance = np.eye(4) * 0.01
        self.autopilot = autopilot
        self.previous_update = None
        self.kf = AdditiveUnscentedKalmanFilter(
            self.additive_transition_function,
            self.additive_observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,

        )

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        if not self.previous_update:
            self.previous_update = time.time()
        self.dt = time.time() - self.previous_update
        c1 = -30
        c2 = -30
        x = (state[0] + (state[1] * self.dt)) + noise[0]
        x_velocity = c1 * ((np.cos(self.autopilot.heading) * np.sin(self.autopilot.angle_x) * np.cos(
            self.autopilot.angle_y)) - (np.sin(0) * np.sin(self.autopilot.angle_y))) + noise[1]
        y = (state[2] + (state[3] * self.dt)) + noise[2]
        y_velocity = c2 * (-np.sin(0) * np.sin(self.autopilot.angle_x) * np.cos(
            self.autopilot.angle_y) - (np.cos(0) * np.sin(self.autopilot.angle_y))) + noise[3]

        self.previous_update = time.time()
        return np.array([x, x_velocity, y, y_velocity])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0],
        ])
        return np.dot(C, state) + noise

    def additive_transition_function(self, state):
        return self.transition_function(state, np.array([0, 0, 0, 0]))

    def additive_observation_function(self, state):
        return self.observation_function(state, np.array([0.001, 0.001]))

    def update_filter(self):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                [self.autopilot.x_distance_to_marker,
                    self.autopilot.y_distance_to_marker],
            )
        )


class UKFPositionOffline2():

    def __init__(self):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        observation_covariance = np.eye(2) * 0.1
        transition_covariance = np.eye(4) * 0.01
        self.previous_update = None
        self.kf = AdditiveUnscentedKalmanFilter(
            self.additive_transition_function,
            self.additive_observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
        )

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        # if not self.previous_update:
        #    self.previous_update = time.time()
       # self.dt = time.time() - self.previous_update
        x = state[0] + (state[1] * self.dt) + noise[0]
        x_velocity = state[1] + noise[1]
        y = (state[2] + (state[3] * self.dt)) + noise[2]
        y_velocity = state[3] + noise[3]
        #y_velocity = c2 * (-np.sin(self.autopilot.heading) * np.sin(self.autopilot.angle_x) * np.cos(self.autopilot.angle_y) - (np.cos(self.autopilot.heading) * np.sin(self.autopilot.angle_y))) + noise[3]
        #x_accelration = c1 * ((np.cos(self.autopilot.heading) * np.sin(self.autopilot.angle_x) * np.cos(self.autopilot.angle_y)) - (np.sin(self.autopilot.heading) * np.sin(self.autopilot.angle_y))) + noise[1]
        #x_velocity = c1 * ((np.cos(self.autopilot.heading) * np.sin(self.autopilot.angle_x) * np.cos(self.autopilot.angle_y)) - (np.sin(self.autopilot.heading) * np.sin(self.autopilot.angle_y))) + noise[1]
        #self.previous_update = time.time()
        return np.array([x, x_velocity, y, y_velocity])

    def observation_function(self, state, noise):
        C = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ])
        return np.dot(C, state) + noise

    def additive_transition_function(self, state):
        return self.transition_function(state, np.array([0.001, 0.001, 0.001, 0.001]))

    def additive_observation_function(self, state):
        return self.observation_function(state, np.array([0.1, 0.1]))

    def update_filter(self, observations):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
            )
        )


class UKFPositionOffline():

    def __init__(self):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        observation_covariance = np.eye(2) * 0.1
        transition_covariance = np.eye(4) * 0.01
        self.previous_update = None
        self.kf = AdditiveUnscentedKalmanFilter(
            self.additive_transition_function,
            self.additive_observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
        )

    def transition_function(self, state, noise):
        x = (state[0] + (state[1] * self.dt)) + noise[0]
        x_velocity = state[1] + noise[1]
        y = (state[2] + (state[3] * self.dt)) + noise[2]
        y_velocity = state[3] + noise[3]
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
