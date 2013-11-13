import time


class Acceleration():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.x = kwargs.get('x', None)
        self.y = kwargs.get('y', None)
        self.z = kwargs.get('z', None)
        self.z_velocity = kwargs.get('z', None)


class SensorModel():

    def __init__(self, serial):
        self.observations = {}
        self.ser = serial

    def update(self, sensor_data):
        #self.observations['acceleration'] = Acceleration()
        self.observations['altitude_barometer'] = float(sensor_data[15])

        self.observations = self.filter_thrust(sensor_data[0])
        self.pitch = self.filter_thrust(sensor_data[1])
        self.yaw = self.filter_thrust(sensor_data[2])
        self.throttle = self.filter_throttle(sensor_data[3])
        self.mode = sensor_data[4]
        self.aux1 = sensor_data[5]
        self.aux2 = sensor_data[6]
        self.armed = sensor_data[7]
        self.angle_x = float(sensor_data[8])
        self.angle_y = float(sensor_data[9])
        self.heading = float(sensor_data[10])
        self.accel_raw_x = float(sensor_data[11])
        self.accel_raw_y = float(sensor_data[12])
        self.accel_raw_z = float(sensor_data[13])
        self.z_velocity = float(sensor_data[14])
        self.altitude_barometer = float(sensor_data[15])
        self.altitude_sonar = float(sensor_data[16])
        self.auto_switch = self.general_filter(sensor_data[17])
        self.battery = sensor_data[18]
        self.flightmode = sensor_data[19]

    def read_sensors(self):
        s = self.ser.readline()
        sensor_data = s.split(',')
        if len(sensor_data) >= 25:
            self.update(sensor_data)
