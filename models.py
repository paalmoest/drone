import time


class Acceleration():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.x = kwargs.get('x', None)
        self.y = kwargs.get('y', None)
        self.z = kwargs.get('z', None)
        self.z_velocity = kwargs.get('z', None)


class SensorModel():

    def __init__(self):
        self.observations = {}

    def update(self, sensor_data):
        self.observations['gyro_x'] = float(sensor_data[8])
        self.observations['gyro_y'] = float(sensor_data[9])
        self.observations['heading'] = float(sensor_data[10])

        self.observations['accel_raw_x'] = float(sensor_data[11])
        self.observations['accel_raw_y'] = float(sensor_data[12])
        self.observations['accel_raw_z'] = float(sensor_data[13])
        self.observations['z_velocity'] = float(sensor_data[14])

        self.observations['altitude_barometer'] = float(sensor_data[15])
        self.observations['altitude_sonar'] = float(sensor_data[16])
        self.observations['battery'] = float(sensor_data[18])
