from pykalman import KalmanFilter
import numpy as np


class StateEstimationAltitude():
    def __init__(self):
        self.state = [0, 0]
        self.transition_covariance = np.array([
            [0.025, 0.005],
            [0.0005, 0.01],
        ])
        self.observation_covariance = np.array([
            [1]
        ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )

    def update(self, observations):
        dt = 0.1
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
                # observation_offset = np.array([, 0, 0])
                # observation_covariance=np.array(0.1*np.eye(1))
            )
        )


class StateEstimation():
    def __init__(self):
        """
        # x,z,y, X,Y,z, Xaccel, yaccel, zaccel
        """
        self.state = [0, 0, 0, 0, 0, 0, 0, 0]
        self.transition_covariance = np.array([
            [0.025, 0.005],
            [0.0005, 0.01],
        ])
        self.observation_covariance = np.array([
            [1]
        ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )

    def update(self, observations):
        dt = 0.1
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
                # observation_offset = np.array([, 0, 0])
                # observation_covariance=np.array(0.1*np.eye(1))
            )
        )
