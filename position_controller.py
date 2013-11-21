from models import SensorModel
from pid import PID
import time


class PIDlog():
    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.correction = kwargs.get('corretion', None)
        self.target = kwargs.get('target', None)
        self.altitude = kwargs.get('altitude', None)
        self.altitude_raw = kwargs.get('altitude_raw', None)
        self.thrust = kwargs.get('thrust', None)


class PositionController():

    def __init__(self, autopilot, state_estimation, **kwargs):
        self.sm = SensorModel()
        self.heading = None
        self.targets = {}
        self.autopilot = autopilot
        self.state_estimation = state_estimation
        self.maximum_thrust = 1950
        self.minimum_thrust = 1450
       # self.altitude_pid = kwargs.get('altitude_pid')
        self.altitude_pid = PID(
            P=0,
            I=0,
            D=0,
            Derivator=0,
            Integrator=0,
            Integrator_max=25,
            Integrator_min=-25
        )
        self.heading_pid = PID(
            P=0,
            I=0,
            D=0,
            Derivator=0,
            Integrator=0,
            Integrator_max=25,
            Integrator_min=-25
        )

    def headingHold(self):
        if not self.targets.get('heading'):
            self.targets['heading'] = self.state_estimation.getHeading()
            self.heading_pid.setPoint(self.targets.get('heading'))
        thrust_correction = self.altitude_pid.update(
            self.state_estimation.getAltitude())
        thrust_correction = self.althold_pid.constraint(thrust_correction)
        self.autopilot.yaw = self.autopilot.yaw + thrust_correction

    def holdAltitude(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.state_estimation.getAltitude()
            self.altitude_pid.setPoint(self.targets.get('altitude'))
        altitude = self.state_estimation.getAltitude()
        thrust_correction = self.altitude_pid.update(altitude)
        thrust_correction = self.altitude_pid.constraint(thrust_correction)
        thrust = self.autopilot.throttle + thrust_correction
        print 'target: %f altitude: %f  corretion: %d current: %d new thrust: %d ' % (self.altitude_pid.set_point, self.state_estimation.getAltitude(), thrust_correction, thrust, self.autopilot.throttle)
        self.autopilot.throttle = self.constraint(thrust)
        self.autopilot.pid_log(
            PIDlog(
                corretion=thrust_correction,
                altitude=altitude,
               # altitude_raw=self.autopilot.altitude_barometer,
                altitude_raw=self.autopilot.altitude_cam,
                target=self.altitude_pid.set_point,
                thrust=thrust,
            )
        )
       # print 'target: %f altitude: %f' % (self.altitude_pid.set_point,
       # self.state_estimation.getAltitude())

    def reset_targets(self):
        self.targets.clear()

    def set_target_altitude(self, altitude):
        self.targets['altitude'] = altitude

    def constraint(self, value):
        if value > self.maximum_thrust:
            return self.maximum_thrust
        elif value < self.minimum_thrust:
            return self.minimum_thrust
        else:
            return int(round(value))

#pc = PositionController(s)
# pc.headingHold()
