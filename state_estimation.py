from pykalman import KalmanFilter
import numpy as np
import time


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


class StateEstimationAltitudeCam():

    def __init__(self):
        self.state = [0, 0]
        self.covariance = np.eye(2)
        self.observation_covariance = np.array([
            [1]
        ])
        self.transition_covariance = np.array([
            [0.001, 0.001],
            [0.001, 0.001],
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
                # observation_offset = np.array([, 0, 0])
                # observation_covariance=np.array(0.1*np.eye(1))
            )
        )
        self.previous_update = time.time()

    def getAltitude(self):
        return self.state[0]


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


class StateEstimationAltitudeCam():

    def __init__(self):
        self.state = [0, 0]
        self.covariance = np.eye(2)
        self.observation_covariance = np.array([
            [1]
        ])
        self.transition_covariance = np.array([
            [0.001, 0.001],
            [0.001, 0.001],
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
                # observation_offset = np.array([, 0, 0])
                # observation_covariance=np.array(0.1*np.eye(1))
            )
        )
        self.previous_update = time.time()

    def getAltitude(self):
        return self.state[0]


class StateEstimationAccel():

    def __init__(self):
        """
        # x,y,z, Xvelo,Yvelo, Zvelo Xaccel, Yaccel, Zaccel, roll, pitch, yaw
        """
        self.state = [0, 0, 0, 0, 0]  # x, y ,z
        self.covariance = np.eye(5)

        transition_covariance = np.array([
                                          [0.01, 0, 0, 0, 0],
                                          [0, 0.1, 0, 0, 0],
                                          [0, 0, 0.001, 0, 0],
                                          [0, 0, 0, 0.001, 0],
                                          [0, 0, 0, 0, 0.001],
                                          ])
        self.observation_covariance = np.eye(3) * 0.1
      #  self.transition_covariance = np.eye(5) * 0.001
        print transition_covariance
        self.kf = KalmanFilter(
            transition_covariance=transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )

    def update(self, dt, observations):
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, dt, 0.5 * (dt ** 2), 0, 0],
                                           [0, 1, dt, 0, 0],
                                           [0, 0, 1, 0, 0],
                                           [0, 0, 0, 1, 0],
                                           [0, 0, 0, 0, 1],
                                           ]),
                observation_matrix=np.array([
                                            [0, 0, 1, 0, 0],
                                            [0, 0, 0, 1, 0],
                                            [0, 0, 0, 0, 1],
                                            ]),
                # observation_offset=np.array([[0]]),
            )
        )


class StateEstimationAltitude_offline():

    def __init__(self):
        self.state = [0, 0]
        self.covariance = np.eye(2)
        self.observation_covariance = np.array([
            [0.01]
        ])
        self.transition_covariance = np.array([
            [0.01, 0],
            [0, 0.01],
        ])
        #self.observation_covariance = np.eye(1) * 1
       # self.transition_covariance = np.eye(2) * 0.001
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, dt, observations):
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

    def getAltitude(self):
        return self.state[0]


class StateEstimationAltitude3():

    def __init__(self):
        self.state = [0, 0, 0]
        self.covariance = np.eye(3)
       # self.observation_offsets = np.array([0,0])
       # self.transition_covariance = np.array([
       #     [0.0000025, 0.000005],
       #     [0.0000005, 0.0000001],
      #  ])

        self.observation_covariance = np.array([
            [0.01, 0],
            [0, 0.01],
        ])
        self.transition_covariance = np.array([
            [0.003, 0.002, 0],
            [0, 0.002, 0.002],
            [0, 0, 0.002],
        ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, dt, observations, u):
        if not self.previous_update:
            self.previous_update = time.time()

        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, dt, 0],
                                           [0, 1, dt],
                                           [0, 0, 1],
                                           ]),
                observation_matrix=np.array([
                                            [1, 0, 0],
                                            [0, 0, 0],
                                            ]),
                transition_offset=np.array([u]),
               # observation_offset=self.observation_offsets,
                # o1bservation_covariance=np.array(0.1*np.eye(1))
            )
        )
        self.previous_update = time.time()

    def getAltitude(self):
        return self.state[0]


class StateEstimationMarker():

    def __init__(self):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
       # self.observation_offsets = np.array([0,0])
       # self.transition_covariance = np.array([
       #     [0.0000025, 0.000005],
       #     [0.0000005, 0.0000001],
      #  ])

        self.observation_covariance = np.array([
            [.5, 0],
            [0, .5],
        ])
        self.transition_covariance = np.array([
            [0.000025, 0, 0.0005, 0],
            [0, 0.000025, 0, 0.0005],
            [0.0005, 0, 0.001, 0],
            [0, 0.000025, 0, 0.001]
            ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, dt, observations, u):
        """
        u = [pitch, roll]
        """
        if not self.previous_update:
            self.previous_update = time.time()
        self.state, self.covariance = (
            self.kf.filter_update(
                self.state,
                self.covariance,
                observations,
                transition_matrix=np.array([
                                           [1, 0, dt, 0],
                                           [0, 1, 0, dt],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]]),
                observation_matrix=np.array([
                                            [1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            ]),
            )
        )
        self.previous_update = time.time()
        return self.state

    def getAltitude(self):
        return self.state[0]


class StateEstimationMarkerOnline():

    def __init__(self):
        self.state = [0, 0, 0, 0]
        self.covariance = np.eye(4)
       # self.observation_offsets = np.array([0,0])
       # self.transition_covariance = np.array([
       #     [0.0000025, 0.000005],
       #     [0.0000005, 0.0000001],
      #  ])

        self.observation_covariance = np.array([
            [.5, 0],
            [0, .5],
        ])
        self.transition_covariance = np.array([
            [0.000025, 0, 0.0005, 0],
            [0, 0.000025, 0, 0.0005],
            [0.0005, 0, 0.001, 0],
            [0, 0.000025, 0, 0.001]
            ])
        self.kf = KalmanFilter(
            transition_covariance=self.transition_covariance,  # H
            observation_covariance=self.observation_covariance,  # Q
        )
        self.previous_update = None

    def update(self, observations):
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
                                           [1, 0, dt, 0],
                                           [0, 1, 0, dt],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]]),
                observation_matrix=np.array([
                                            [1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            ]),
            )
        )
        self.previous_update = time.time()
        return self.state

    def getXposition(self):
        return self.state[0]

    def getYposition(self):
        return self.state[1]

    def getXVelocity(self):
        return self.state[2]

    def getYVelocity(self):
        return self.state[3]
