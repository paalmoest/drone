import time


class PID:

    """
    Discrete PID control
    """

    def __init__(self, **kwargs):

        self.Kp = kwargs.get('P', 0)
        self.Ki = kwargs.get('I', 0)
        self.Kd = kwargs.get('D', 0)
        self.Derivator = kwargs.get('Derivator', 0)
        self.Integrator = kwargs.get('Integrator', 0)
        self.Integrator_max = kwargs.get('Integrator_max', 25)
        self.Integrator_min = kwargs.get('Integrator_min', 25)

        self.set_point = 0.0
        self.error = 0.0

        self.minimum_thrust = kwargs.get('minimum_thrust', -50)
        self.maximum_thrust = kwargs.get('maximum_thrust', 50)

        self.current_time = time.time()
        self.previous_time = time.time()

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value
        self.current_time = time.time()
        dt = self.current_time - self.previous_time

        if dt > 0:
            self.Derivator = (self.error - self.previous_error) / dt
        else:
            self.Derivator = 0
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * self.Derivator
       # self.D_value = self.Kd * (self.error - self.Derivator)
        self.Integrator += self.error * dt

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value
        self.previous_error = self.error
        self.previous_time = time.time()

        return PID

    def setPoint(self, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator = 0
        self.Derivator = 0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

    def constraint(self, value):
        if value > self.maximum_thrust:
            return self.maximum_thrust
        elif value < self.minimum_thrust:
            return self.minimum_thrust
        else:
            return int(round(value))
