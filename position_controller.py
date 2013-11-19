from models import SensorModel
from pid import PID


class PositionController():
    def __init__(self, state_estimation, **kwargs):
        self.sm = SensorModel()
        self.heading = None
        self.targets = {}
        self.state_estimation = state_estimation
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
        thrust_correction = self.altitude_pid.update(
            self.state_estimation.getAltitude())
        thrust_correction = self.althold_pid.constraint(thrust_correction)
        self.autopilot.throttle = self.autopilot.throttle + thrust_correction
        #self.autopilot.send_control_commands()

    def set_target_altitude(self, altitude):
        self.targets['altitude'] = altitude


#pc = PositionController(s)
#pc.headingHold()
