class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=25, I=0.6, D=0, Derivator=0, Integrator=0, Integrator_max=1000, Integrator_min=-1000, **kwargs):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.set_point = 0.0
        self.error = 0.0

        self.minimum_thrust = kwargs.get('minimum_thrust', -50)
        self.maximum_thrust = kwargs.get('maximum_thrust', 70)

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

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

    def constraint(self, pid):
        if pid > self.maximum_thrust:
            return self.maximum_thrust
        elif pid < self.minimum_thrust:
            return self.minimum_thrust
        else:
            return int(round(pid))

"""
pid = PID()

pid.setPoint(2.00)


print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
print pid.constraint(pid.update(1.0))
#print
#print pid.update(1.10)
#print pid.update(2.04)
#rint pid.update(1.80)



#print pid.update(2.12)"""
