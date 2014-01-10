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

    def __init__(self,
                 autopilot,
                 state_estimation,
                 **kwargs):
        self.sm = SensorModel()
        self.heading = None
        self.targets = {}
        self.autopilot = autopilot
        self.state_estimation = state_estimation
        self.position_hold_init = False
        self.heading_pid = kwargs.get('heading_pid', None)
        self.roll_pid = kwargs.get('roll_pid', None)
        self.pitch_pid = kwargs.get('pitch_pid', None)
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
            self.yaw_hold_init = self.autopilot.yaw
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

    def altitudeHoldSonar(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.autopilot.altitude_sonar
            self.altitude_pid.setPoint(self.targets.get('altitude'))
            #self.altitude_pid.setPoint(2.0)
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
        """
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
        """
        self.autopilot.throttle = self.altitude_hold_throttle + correction
        self.log_altitude(altitude, correction)

    def battery_correction(self):
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
                target=self.heading_pid.getpoint(),
                thrust=self.autopilot.yaw,
                error=self.heading_pid.error,
                intergator=self.heading_pid.getIntegrator(),
                corretion=corretion,
                P_corretion=self.heading_pid.P_value,
                I_corretion=self.heading_pid.I_value,
                D_corretion=self.heading_pid.D_value,
            )
        )

    def log_roll(self, observation, correction):
        self.autopilot.pid_log_roll.append(
            PIDlog_generic(
                observation=observation,
                target=self.roll_pid.getPoint(),
                thrust=self.autopilot.roll,
                error=self.roll_pid.error,
                intergator=self.roll_pid.getIntegrator(),
                corretion=correction,
                P_corretion=self.roll_pid.P_value,
                I_corretion=self.roll_pid.I_value,
                D_corretion=self.roll_pid.D_value,
            )
        )

    def log_pitch(self, observation, correction):
        self.autopilot.pid_log_pitch.append(
            PIDlog_generic(
                observation=observation,
                target=self.pitch_pid.getpoint(),
                thrust=self.autopilot.pitch,
                error=self.pitch_pid.error,
                intergator=self.pitch_pid.getIntegrator(),
                corretion=correction,
                P_corretion=self.pitch_pid.P_value,
                I_corretion=self.pitch_pid.I_value,
                D_corretion=self.pitch_pid.D_value,
            )
        )

    def positionHold(self):
        if not self.position_hold_init:
            self.position_hold_roll = self.autopilot.roll
            self.position_hold_pitch = self.autopilot.pitch
            self.roll_pid.setPoint(0.0)
            self.position_hold_init = True
            print "init"

        x = self.autopilot.linear_position.getPositionX()
        y = self.autopilot.linear_position.getPositionY()

        roll_correction = self.roll_pid.update(x)
        pitch_correction = self.pitch_pid.update(y)
        self.autopilot.roll = self.constraint(self.position_hold_roll + roll_correction)
        self.autopilot.pitch = self.constraint(self.position_hold_pitch + pitch_correction)

        self.log_roll(x, roll_correction)
        self.log_pitch(y, pitch_correction)
        print 'x: %0.3f y: %0.3f roll: %d roll correction: %d pitch: %d pitch correction: %d' % (
            x,
            y,
            self.autopilot.roll,
            roll_correction,
            self.autopilot.pitch,
            pitch_correction,
        )
   
    def constraint(self, value):
        if value >= 1600:
            return 1600
        elif value <= 1400:
            return 1400
        else:
            return int(round(value))

    def reset_targets(self):
        self.targets.clear()
        self.heading_pid.reset()
        self.position_hold_init = False

    def set_target_altitude(self, altitude):
        self.targets['altitude'] = altitude

    def yaw_constraint(self, value):
        if value >= 1750:
            return 1750
        elif value <= 1250:
            return 1250
        else:
            return int(round(value))
