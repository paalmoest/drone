from pykalman import UnscentedKalmanFilter
import numpy as np
import time

class UKFPosition():

    def __init__(self, autopilot):
        self.state = [0, 0, 0, 0, 0, 0, 0]
        self.covariance = np.eye(7)
        observation_covariance = np.eye(5) * 0.5
        transition_covariance = np.eye(7) * 0.001
        self.autopilot = autopilot
        self.kf = UnscentedKalmanFilter(
            self.transition_function, self.observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            random_state=np.random.RandomState(0),

        )
        self.previous_update = None

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        if not self.previous_update:
            self.previous_update = time.time()
        self.dt = time.time() - self.previous_update
        c1 = -0.1
        roll = state[4] + noise[4]
        pitch = state[5] + noise[5]
        yaw = state[6] + noise[6]
        x = (state[0] + (state[1] * self.dt)) + noise[0]
        x_velocity = c1 * (np.cos(yaw) * np.sin(roll) * np.cos(pitch) - np.sin(yaw) * np.sin(pitch)) + noise[1]
        y = (state[2] + (state[3] * self.dt)) + noise[2]
        y_velocity = c2 * (-np.sin(yaw) * np.sin(roll) * np.cos(pitch) - np.cos(yaw) * np.sin(pitch)) + noise[3]
        self.previous_update = time.time()
        return np.array([x, x_velocity, y, y_velocity, roll, pitch, yaw])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1],
        ])
        return np.dot(C, state) + noise

    def update_filter(self):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                np.asarray([
                    self.autopilot.x_distance_to_marker,
                    self.autopilot.angle_x,
                    self.autopilot.angle_y,
                    self.autopilot.heading,
                ])
            )
        )
