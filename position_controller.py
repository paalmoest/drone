from models import SensorModel
from pid import PID
import time
import math


class MetaPid():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.P = kwargs.get('P', None)
        self.I = kwargs.get('I', None)
        self.D = kwargs.get('D', None)
        self.maximum_thrust = kwargs.get('maximum_thrust', None)
        self.minimum_thrust = kwargs.get('minimum_thrust', None)


class PIDlog_generic():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.P_corretion = kwargs.get('P_corretion', None)
        self.I_corretion = kwargs.get('I_corretion', None)
        self.D_corretion = kwargs.get('D_corretion', None)
        self.correction = kwargs.get('corretion', None)
        self.target = kwargs.get('target', None)
        self.observation = kwargs.get('observation', None)
        self.thrust = kwargs.get('thrust', None)
        self.error = kwargs.get('error', None)
        self.intergator = kwargs.get('intergator', None)


class PositionController():

    def __init__(self, autopilot, state_estimation, **kwargs):
        self.sm = SensorModel()
        self.heading = None
        self.targets = {}
        self.autopilot = autopilot
        self.state_estimation = state_estimation
        self.heading_pid = kwargs.get('heading_pid', None)
        self.altitude_pid = kwargs.get(
            'altitude_pid', self.pidFactory(P=25, I=0, D=0))

    def pidFactory(self, **kwargs):
        return PID(
            P=kwargs.get('P', 1),
            I=kwargs.get('I', 0),
            D=kwargs.get('D', 0),
            Derivator=0,
            Integrator=0,
            Integrator_max=25,
            Integrator_min=-25,
            maximum_thrust=50,
            minimum_thrust=-50,
        )

    def headingHold(self):
        if not self.targets.get('heading'):
            self.targets['heading'] = self.autopilot.heading
            self.heading_pid.setPoint(self.targets.get('heading'))
            self.autopilot.meta_pid = MetaPid(
                P=self.heading_pid.Kp,
                I=self.heading_pid.Ki,
                D=self.heading_pid.Kd,
                maximum_thrust=self.heading_pid.maximum_thrust,
                minimum_thrust=self.heading_pid.minimum_thrust,
            )
        thrust_correction = self.heading_pid.update(self.autopilot.heading)
        thrust = self.autopilot.yaw + thrust_correction
        thrust = self.yaw_constraint(thrust)
        #self.print_heading(thrust_correction, thrust)
        self.log_heading(thrust_correction)
        self.autopilot.yaw = thrust

    def print_heading(self, thrust_correction, thrust):
        print 'target: %f heading: %f  corretion: %d current yaw: %d new yaw: %d ' % (
            self.heading_pid.set_point,
            self.autopilot.heading,
            thrust_correction,
            self.autopilot.yaw,
            thrust)

    def new_heading(self, value):
        self.heading_pid.setPoint(
            self.heading_pid.set_point + math.radians(value)
        )

    def altitudeHold(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.state_estimation.getAltitude()
            self.altitude_pid.setPoint(self.targets.get('altitude'))
            # self.altitude_pid.setPoint(2)
            self.autopilot.meta_pid_alt = MetaPid(
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
        thrust = self.altitude_pid.throttle_constraint(thrust)
        print 'target: %f altitude: %f  corretion: %d current: %d new thrust: %d P: %d I: %d D: %d' % (self.altitude_pid.set_point, self.state_estimation.getAltitude(), thrust_correction, self.autopilot.throttle, thrust, self.altitude_pid.Kp,self.altitude_pid.Ki, self.altitude_pid.Kd) 
        self.log_altitude(altitude, thrust_correction)
        self.autopilot.throttle = thrust

    def log_altitude(self, altitude, correction):
        self.autopilot.pid_log_altitudeHold.append(
            PIDlog_generic(
                observation=altitude,
                target=self.altitude_pid.set_point,
                thrust=self.autopilot.throttle,
                error=self.altitude_pid.error,
                intergator=self.altitude_pid.getIntegrator(),
                corretion=correction,
                P_corretion=self.altitude_pid.P_value,
                I_corretion=self.altitude_pid.I_value,
                D_corretion=self.altitude_pid.D_value,
            )
        )

    def log_heading(self, corretion):
        self.autopilot.pid_log.append(
            PIDlog_generic(
                observation=self.autopilot.heading,
                target=self.heading_pid.set_point,
                thrust=self.autopilot.yaw,
                error=self.heading_pid.error,
                intergator=self.heading_pid.getIntegrator(),
                corretion=corretion,
                P_corretion=self.heading_pid.P_value,
                I_corretion=self.heading_pid.I_value,
                D_corretion=self.heading_pid.D_value,
            )
        )

    def positionHold(self):
        x = self.state_estimation_marker.getX()
        y = self.state_estimation_marker.getX()
        roll_correction = self.roll_pid.update(x)
        pitch_correction = self.pitch_pid.update(y)
        self.autopilot.pitch += roll_correction
        self.autopilot += pitch_correction

    def reset_targets(self):
        self.targets.clear()
        self.heading_pid.reset()

    def set_target_altitude(self, altitude):
        self.targets['altitude'] = altitude

    def yaw_constraint(self, value):
        if value >= 1750:
            return 1750
        elif value <= 1250:
            return 1250
        else:
            return int(round(value))


#pc = PositionController(s)
# pc.headingHold()
