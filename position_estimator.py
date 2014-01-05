from pykalman import AdditiveUnscentedKalmanFilter, KalmanFilter
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
            random_state=np.random.RandomState(0),
        )

    def transition_function(self, state, noise):
        if not self.previous_update:
            self.previous_update = time.time()
        self.dt = time.time() - self.previous_update
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

    def update_filter(self, roll, pitch, yaw):
        observations = self.calculateHorizontalVelocity(roll, pitch, yaw)
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
        c1 = -30
        c2 = -30
        x_v = c1 * \
            (np.cos(yaw) * np.sin(roll) * np.cos(pitch)
             - np.sin(yaw) * np.sin(pitch))
        y_v = c2 * \
            (-np.sin(yaw) * np.sin(roll) *
             np.cos(pitch) - np.cos(yaw) * np.sin(pitch))
        return [x_v, y_v]


class UKFPosition2():

    def __init__(self, autopilot):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
        observation_covariance = np.eye(4) * 0.1
        transition_covariance = np.eye(4) * 0.01
        self.init_yaw = 0
        self.autopilot = autopilot
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

        self.dt = time.time() - self.previous_update

        x = (state[0] + (state[1] * self.dt)) + noise[0]
        x_velocity = state[1] + noise[1]
        y = (state[2] + (state[3] * self.dt)) + noise[2]
        y_velocity = state[3] + noise[3]

        self.previous_update = time.time()

        return np.array([x, x_velocity, y, y_velocity])

    def observation_function(self, state, noise):
        C = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        return np.dot(C, state) + noise

    def additive_transition_function(self, state):
        return self.transition_function(state, np.array([0, 0, 0, 0]))

    def additive_observation_function(self, state):
        return self.observation_function(state, np.array([0, 0, 0, 0]))

    def update_filter(self):
        yaw = 0
        c1 = -1
        x_v = c1 * (np.cos(yaw) * np.sin(self.autopilot.angle_x) * np.cos(
            self.autopilot.angle_y) - (np.sin(yaw) * np.sin(self.autopilot.angle_y)))
        y_v = c1 * (-np.sin(yaw) * np.sin(self.autopilot.angle_x) * np.cos(
            self.autopilot.angle_y - np.cos(yaw) * np.sin(self.autopilot.angle_y)))
        observations = [
            self.autopilot.x_distance_to_marker,
            x_v,
            self.autopilot.y_distance_to_marker,
            y_v
        ]
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
            )
        )

    def getPositionX(self):
        return self.state[0]

    def getPositionY(self):
        return self.state[1]

    def getSpeedX(self):
        return self.state[2]

    def getSpeedY(self):
        return self.state[3]


class LinearPosition():

    def __init__(self):
        self.state = [0, 0, 0, 0, 0, 0]
        self.covariance = np.eye(6)
        self.observation_covariance = np.array([
            [.5, 0],
            [0, .5],
        ])
        self.transition_covariance = np.eye(6) * .001
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, observations, u):
        """
        u = [pitch, roll]
        """
        if not self.previous_update:
            self.previous_update = time.time()
        dt = time.time() - self.previous_update
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, 0, dt, 0, 0.5 * (dt ** 2), 0],
                                           [0, 1, 0, dt, 0, 0.5 * (dt ** 2)],
                                           [0, 0, 1, 0, dt, 0],
                                           [0, 0, 0, 1, 0, dt],
                                           [0, 0, 0, 0, 1, 0],
                                           [0, 0, 0, 0, 0, 1],
                                           ]),
                observation_matrix=np.array([
                                            [1, 0, 0, 0, 0, 0],
                                            [0, 1, 0, 0, 0, 0],
                                            ]),
               # transition_offset=u,
            )
        )
        self.previous_update = time.time()
        print 'x: %0.3f  y: %0.3f' % (self.getPositionX(), self.getPositionY())

    def getPositionX(self):
        return self.state[0]

    def getPositionY(self):
        return self.state[1]

    def getSpeedX(self):
        return self.state[2]

    def getSpeedY(self):
        return self.state[3]
