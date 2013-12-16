from models import SensorModel
from pid import PID
import time
import math
import numpy as np

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

    def __init__(self, autopilot, state_estimation, state_estimation_marker, **kwargs):
        self.sm = SensorModel()
        self.heading = None
        self.targets = {}
        self.autopilot = autopilot
        self.state_estimation = state_estimation
        self.state_estimation_marker = state_estimation_marker
        self.position_hold_init = False
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
        thrust = self.autopilot.throttle_alt + thrust_correction
        thrust = self.altitude_pid.throttle_constraint(thrust)
        print 'target: %f altitude: %f  corretion: %d current: %d new thrust: %d P: %d I: %d D: %d' % (self.altitude_pid.set_point, self.state_estimation.getAltitude(), thrust_correction, self.autopilot.throttle, thrust, self.altitude_pid.Kp, self.altitude_pid.Ki, self.altitude_pid.Kd)
        self.log_altitude(altitude, thrust_correction)
        self.autopilot.throttle = thrust

    def altitudeHoldSonar(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.autopilot.altitude_sonar
            self.altitude_pid.setPoint(self.targets.get('altitude'))
            # self.altitude_pid.setPoint(2)
            self.autopilot.meta_pid_alt = MetaPid(
                P=self.altitude_pid.Kp,
                I=self.altitude_pid.Ki,
                D=self.altitude_pid.Kd,
                maximum_thrust=self.altitude_pid.maximum_thrust,
                minimum_thrust=self.altitude_pid.minimum_thrust,
            )
        altitude = self.autopilot.altitude_sonar
        thrust_correction = self.altitude_pid.update(altitude)
        thrust_correction = thrust_correction * 100
        thrust_correction = self.altitude_pid.constraint(thrust_correction)
        thrust = self.autopilot.throttle + thrust_correction
        thrust = self.altitude_pid.throttle_constraint(thrust)
        print 'target: %f altitude: %f  corretion: %d current: %d new thrust: %d P: %d I: %d D: %d' % (self.altitude_pid.set_point, self.state_estimation.getAltitude(), thrust_correction, self.autopilot.throttle, thrust, self.altitude_pid.Kp, self.altitude_pid.Ki, self.altitude_pid.Kd)
        self.log_altitude(altitude, thrust_correction)
        self.autopilot.throttle = thrust

    def altitudeHoldSonarKalman(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.state_estimation.getAltitude()
            self.altitude_pid.setPoint(self.targets.get('altitude'))
            self.altitude_hold_throttle = self.autopilot.throttle
            self.altitude_hold_battery = self.autopilot.battery
            # self.altitude_pid.setPoint(2)
            self.autopilot.meta_pid_alt = MetaPid(
                P=self.altitude_pid.Kp,
                I=self.altitude_pid.Ki,
                D=self.altitude_pid.Kd,
                maximum_thrust=self.altitude_pid.maximum_thrust,
                minimum_thrust=self.altitude_pid.minimum_thrust,
            )
        altitude = self.state_estimation.getAltitude()
        correction = self.altitude_pid.update(altitude)
        correction = self.altitude_pid.constraint(correction)
        throttle = self.altitude_hold_throttle + correction
        #throttle = self.altitude_hold_throttle * self.batter_correction() + correction

        print 'target: %.2f altitude: %.2f  corretion: %d current: %d new thrust: %d P: %.2f I: %.2f D: %.2f battery: %.2f'  % (
            self.altitude_pid.set_point,
            self.state_estimation.getAltitude(),
            correction,
            self.autopilot.throttle,
            throttle,
            self.altitude_pid.Kp,
            self.altitude_pid.Ki,
            self.altitude_pid.Kd,
            self.autopilot.battery,
            )
        self.autopilot.throttle = self.altitude_hold_throttle + correction
        self.log_altitude(altitude, correction)

    def batter_correction(self):
        return (self.altitude_hold_battery / self.autopilot.battery)

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
                D_corretion=self.altitude_pid.D_value,
                I_corretion=self.altitude_pid.I_value,
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

    def calcualte_xDistance(self):
        camera_x_center = 80
        z = self.autopilot.altitude_sonar * np.cos(self.angle_x)
        l = np.sin(self.autopilot.angle_x) * z
        pixels_per_meter = (121.742 / z)
        x_diff_pixels = camera_x_center - self.state_estimation_marker.getXposition()
        x = (x_diff_pixels / pixels_per_meter)
        m = l - x
        return m


    def positionHold(self):
        if not self.position_hold_init:
            self.autopilot.position_hold_roll = self.autopilot.roll
            self.autopilot.position_hold_pitch = self.autopilot.pitch
            self.roll_pid.setPoint(0.0)
            self.position_hold_init = True
        x_position = self.state_estimation_marker.getXposition()
        y_position = self.state_estimation_marker.getYposition()
        print 'x: %d y: %d x_attitude: %f y_attitude: %f ' % (x_position, y_position, self.autopilot.angle_x, self.autopilot.angle_y)
        #roll_correction = self.roll_pid.update(x_distance)
        #pitch_correction = self.pitch_pid.update(y_distance)
        #self.autopilot.roll = self.autopilot.position_hold_pitch + roll_correction
        #self.autopilot.pitch = self.autopilot.position_hold_roll + pitch_correction



    def reset_targets(self):
        self.targets.clear()
        self.heading_pid.reset()
        self.position_hold_init = True

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
