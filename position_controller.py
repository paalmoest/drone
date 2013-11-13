from models import SensorModel


class PositionController():

    def __init__(self, **kwargs):
        self.sm = SensorModel()
        self.heading = None
        self.targets = {}
        self.altitude_pid = kwargs.get('altitude_pid')

    def headingHold(self):
        if not self.targets.get('heading'):
            self.heading = self.sm.observations.get('heading')
        print self.heading

    def alitudeHold(self):
        if not self.targets.get('altitude'):
            self.targets['altitude'] = self.sm.observations.get('altitude')
            self.altitude_pid.set_point(self.targets.get('altitude'))
        thrust_correction = self.altitude_pid.update(
            self.sm.observations.get('altitude'))
        thrust_correction = self.althold_pid.constraint(thrust_correction)
        self.autopilot.throttle = self.autopilot.throttle + thrust_correction
        self.autopilot.send_control_commands()


pc = PositionController()
pc.headingHold()
