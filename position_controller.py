from models import SensorModel
from pid import PID
import time


class MetaPid():
    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.P = kwargs.get('P', None)
        self.I = kwargs.get('I', None)
        self.D = kwargs.get('D', None)
        self.maximum_thrust = kwargs.get('maximum_thrust', None)
        self.minimum_thrust = kwargs.get('minimum_thrust', None)


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
        self.maximum_thrust = 1850
        self.minimum_thrust = 1350
       # self.altitude_pid = kwargs.get('altitude_pid')
        p = 25
        i = 0.6
        d = 0
        max_t = 50
        min_t = -5

        self.altitude_pid = PID(
            P=p,
            I=i,
            D=d,
            Derivator=0,
            Integrator=0,
            Integrator_max=25,
            Integrator_min=-25,
            maximum_thrust=max_t,
            minimum_thrust=min_t,
        )
        self.heading_pid = PID(
            P=1,
            I=0,
            D=0,
            Derivator=0,
            Integrator=0,
            Integrator_max=25,
            Integrator_min=-25,
            maximum_thrust=25,
            minimum_thrust=-25,
        )

    def headingHold(self):
        if not self.targets.get('heading'):
            #self.targets['heading'] = self.state_estimation.getHeading()
            self.targets['heading'] = self.autopilot.heading
            self.heading_pid.setPoint(self.targets.get('heading'))
        thrust_correction = self.altitude_pid.update(self.autopilot.heading)
       # thrust_correction = self.althold_pid.constraint(thrust_correction)
        self.autopilot.yaw = self.autopilot.yaw + thrust_correction

    def holdAltitude(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.state_estimation.getAltitude()
            self.altitude_pid.setPoint(self.targets.get('altitude'))
            self.autopilot.meta_pid = MetaPid(
                P=self.altitude_pid.Kp,
                I=self.altitude_pid.Ki,
                D=self.altitude_pid.Kd,
                maximum_thrust=self.altitude_pid.maximum_thrust,
                minimum_thrust=self.altitude_pid.minimum_thrust,
                )
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
                #  altitude_raw=self.autopilot.altitude_barometer,
                altitude_raw=self.autopilot.altitude_barometer,
                target=self.altitude_pid.set_point,
                thrust=self.autopilot.throttle,
                error=self.altitude_pid.error,
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
