from pykalman import KalmanFilter
import numpy as np
import time


class PositionEstimation():

    def __init__(self):
        self.state = [0, 0, 0, 0, 0, 0]
        self.covariance = np.eye(6)
        self.observation_covariance = np.eye(4) * 1
        self.transition_covariance = np.eye(6) * 0.01
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, dt, observations):
        print observations
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
                                            [0, 0, 1, 0, 0, 0],
                                            ]),
            )
        )
        self.previous_update = time.time()

    def getAltitude(self):
        return self.state[0]


class StateEstimate():

    def __init__(self):
        self.state = [0, 0, 0]
        self.covariance = np.eye(3)
        self.observation_covariance = np.eye(3) * 0.5
        self.transition_covariance = np.eye(3) * 0.01
        self.previous_update = None

    def update(self, observations):
        if not self.previous_update:
            self.previous_update = time.time()
        self.dt = time.time() - self.previous_update

        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, 0, 0],
                                           [0, 1, 0],
                                           [0, 0, 1],
                                           ]),
                observation_matrix=np.array([
                                            [1, 0, 0],
                                            [0, 1, 0],
                                            [0, 0, 1],
                                            ]),
            )
        )
        self.previous_update = time.time()

    def getPitchAngle(self):
        return self.state[0]

    def getRollAngle(self):
        return self.state[0]

    def getYawAngle(self):
        return self.state[0]


class StateEstimationAltitude():

    def __init__(self):
        self.state = [0, 0]
        self.covariance = np.eye(2)
        self.observation_covariance = np.array([
            [1]
        ])
        self.transition_covariance = np.array([
            [0.001, 0],
            [0, 0.001],
        ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, observations):
        if not self.previous_update:
            self.previous_update = time.time()
        dt = time.time() - self.previous_update

        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, dt],
                                           [0, 1]
                                           ]),
                observation_matrix=np.array([
                                            [1, 0],
                                            ]),
            )
        )
        self.previous_update = time.time()

    def getAltitude(self):
        return self.state[0]
    
    def getVelocity(self):
        return self.state[1]


class StateEstimationAltitudeSonar():

    def __init__(self):
        self.state = [0, 0]
        self.covariance = np.eye(2)
        self.observation_covariance = np.array([
            [0.1]
        ])
        self.transition_covariance = np.array([
            [0.01, 0],
            [0, 0.01],
        ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, dt, observations):
        if not self.previous_update:
            self.previous_update = time.time()
        #dt = time.time() - self.previous_update

        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, dt],
                                           [0, 1]
                                           ]),
                observation_matrix=np.array([
                                            [1, 0],
                                            ]),
            )
        )
        self.previous_update = time.time()

    def getAltitude(self):
        return self.state[0]

    def getVelocity(self):
        return self.state[1]


class Attitude():

    def __init__(self):
        self.state = [0, 0, 0]
        self.covariance = np.eye(3)
        self.observation_covariance = np.array([
            [.1, 0, 0],
            [0, .1, 0],
            [0, 0, .1],
        ])
        self.transition_covariance = np.eye(3) * 0.01
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, observations):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, 0, 0],
                                           [0, 1, 0],
                                           [0, 0, 1],
                                           ]),
                observation_matrix=np.array([
                                            [1, 0, 0],
                                            [0, 1, 0],
                                            [0, 0, 1],
                                            ]),
            )
        )
        return self.state
