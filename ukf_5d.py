from pykalman import UnscentedKalmanFilter
import numpy as np

class UKFPosition():

    def __init__(self, autopilot):
        self.state = [0, 0, 0, 0, 0]
        self.covariance = np.eye(5)
        observation_covariance = np.eye(4) * 0.5
        transition_covariance = np.eye(5) * 0.001
        self.autopilot = autopilot
        self.kf = UnscentedKalmanFilter(
            self.transition_function, self.observation_function,
            observation_covariance=observation_covariance,
            transition_covariance=transition_covariance,
            random_state=np.random.RandomState(0),

        )

    def transition_function(self, state, noise):
        #a = state[0] + state[1] * self.t * noise[0]
        c1 = 1
        c2 = 1
        roll = state[2] + noise[2]
        pitch = state[3] + noise[3]
        yaw = state[4] + noise[4]
        b = c1 * (c2 * ((np.cos(yaw) * np.sin(roll) * np.cos(pitch)) - ((np.sin(yaw) * np.sin(pitch))))) + noise[1]
        a = (state[0] + (b * self.dt)) + noise[0]
        return np.array([a, b, roll, pitch, yaw])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
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
