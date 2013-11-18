import serial
import time
import datetime
import pickle
import os
from pid import PID

# sample
# 1,0.02,-0.01,1.16,-0.19,0,1518,1497,1498,1590,1935,1969,0,0,1597,1587,1579,1577,0,0,0,0,10.20,1,


class Sensor():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.value = kwargs.get('value', None)


class Acceleration():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.x = kwargs.get('x', None)
        self.y = kwargs.get('y', None)
        self.z = kwargs.get('z', None)
        self.z_velocity = kwargs.get('z', None)


class Attitude():

    def __init__(self, **kwargs):
        self.roll = kwargs.get('roll', None)
        self.pitch = kwargs.get('pitch', None)
        self.yaw = kwargs.get('yaw', None)
        self.timestamp = time.time()


class PositionController():

    def _init_(self):
        pass


class AutoPilot():

    def __init__(self,  state_estimate, position_controller, **kwargs):
        simulate = kwargs.get('simulate', False)
        if not simulate:
            self.connect_to_drone()
        self.state_estimate = state_estimate
        self.position_controller = position_controller

        self.pixel_threshold = kwargs.get('pixel_threshold', 100)
        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.vebrose = kwargs.get('vebrose')
        self.cam_center = [self.cam_width / 2, self.cam_height / 2]

        self.auto_switch = False
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = 1500
        self.altitudehold = False
        self.altitude_sonar = 0.00
        self.altitude_barometer = 0.00
        self.alitude_target = None
        self._altitudehold = ''
        self.z_velocity = 0.0
        self.mode = ''
        self.aux1 = ''
        self.aux2 = ''
        self.accel_raw_x = 0.0
        self.accel_raw_y = 0.0
        self.accel_raw_z = 0.0
        self.left = True
        self.previous_time = time.time()

        self.roll_thrust = 1500
        self.pitch_thrust = 1500
        self.init_thrust = 1500
        self.direction = True

        self.init_logging()

    def init_logging(self):
        self.roll_array = []
        self.pitch_array = []
        self.yaw_array = []
        self.throttle_array = []
        self.sonar_array = []
        self.baro_array = []
        self.thrust_correction = []
        self.maker_positions = []
        self.attitude = []
        self.acceleration = []
        self.z_velocity_array = []

    def log(self):
        self.pitch_array.append((time.time(), self.pitch))
        self.roll_array.append((time.time(), self.roll))
        self.yaw_array.append((time.time(), self.yaw))
        self.throttle_array.append((time.time(), self.throttle))
        self.sonar_array.append((time.time(), self.altitude_sonar))
        self.baro_array.append((time.time(), self.altitude_barometer))
        self.z_velocity_array.append(Sensor(value=self.z_velocity))
        self.attitude.append(
            Attitude(roll=self.angle_x, pitch=self.angle_y, yaw=self.heading))
        self.acceleration.append(
            Acceleration(
                x=self.accel_raw_x,
                y=self.accel_raw_y,
                z=self.accel_raw_z,
                z_velocity=self.z_velocity
            )
        )

    def get_test_number(self, mypath, number):
        tmp = mypath + str(number)
        if not os.path.isdir('data/%s' % tmp):
            return tmp
        else:
            return self.get_test_number(mypath, number + 1)

    def dump_log(self):
        mypath = 'test_'
        number = 1
        mypath = self.get_test_number(mypath, number)
        os.makedirs('data/%s' % mypath)
        pickle.dump(self.roll_array, open(
            'data/%s/%s.dump' % (mypath, 'roll'), 'wb'))
        pickle.dump(self.pitch_array, open(
            'data/%s/%s.dump' % (mypath, 'pitch'), 'wb'))
        pickle.dump(self.yaw_array, open(
            'data/%s/%s.dump' % (mypath, 'yaw'), 'wb'))
        pickle.dump(self.throttle_array, open(
            'data/%s/%s.dump' % (mypath, 'throttle'), 'wb'))
        pickle.dump(self.sonar_array, open(
            'data/%s/%s.dump' % (mypath, 'sonar'), 'wb'))
        pickle.dump(self.baro_array, open(
            'data/%s/%s.dump' % (mypath, 'barometer'), 'wb'))
        pickle.dump(self.thrust_correction, open(
            'data/%s/%s.dump' % (mypath, 'thrust_correction'), 'wb'))
        pickle.dump(self.acceleration, open(
            'data/%s/%s.dump' % (mypath, 'acceleration'), 'wb'))
        pickle.dump(
            self.attitude, open('data/%s/%s.dump' % (mypath, 'attitude'), 'wb'))
        pickle.dump(self.maker_positions, open(
            'data/%s/%s.dump' % (mypath, 'marker_positions'), 'wb'))
        exit()

    def connect_to_drone(self):
        self.ser = serial.Serial(
            port='/dev/ttyACM0', baudrate=115200, timeout=1)
        self.ser.open()
        string = 'connect_to_drone .'
        wait = 5
        for x in range(wait):
            string += '.'
            print string
            time.sleep(1)

        self.ser.write('7')

    def run(self):
        sensor_data = self.read_sensors()
        if sensor_data:
            self.set_state(sensor_data)
        # print self.get_copter_state(sensor_data)

    def update_marker(self, marker):
        if marker:
            self.altitude_camera = marker.get_altitude()
        self.maker_positions.append(marker)

    def update_state(self, data):
        self.roll = self.filter_thrust(data[0])
        self.pitch = self.filter_thrust(data[1])
        self.yaw = self.filter_thrust(data[2])
        self.throttle = self.filter_throttle(data[3])
        self.mode = data[4]
        self.aux1 = data[5]
        self.aux2 = data[6]
        self.armed = data[7]
        self.angle_x = float(data[8])
        self.angle_y = float(data[9])
        self.heading = float(data[10])
        self.accel_raw_x = float(data[11])
        self.accel_raw_y = float(data[12])
        self.accel_raw_z = float(data[13])
        self.z_velocity = float(data[14])
        self.altitude_barometer = float(data[15])
        self.altitude_sonar = float(data[16])
        self.auto_switch = self.general_filter(data[17])
        self.battery = data[18]
        self.flightmode = data[19]

        self.state_estimation.update(self.altitude_barometer)
        print self.altitude_barometer
        print self.state_estimation.getAltitude()

        # self.log()
    def print_commands(self):
        return 'roll %d \
                pitch %d \
                yaw %d \
                throttle %d \
                auto %d ' % (
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
            self.auto_switch)

    def _read_sensors(self):
        s = self.ser.readline()
        print s
        sensor_data = s.split(',')
        if len(sensor_data) >= 25:
            self.update_state(sensor_data)

    def read_sensors(self):
        s = self.ser.readline()
        sensor_data = s.split(',')
        if len(sensor_data) >= 25:
            self.set_state(sensor_data)

    def enable_altitudehold(self):
        string = 'Q%s;%s' % (str(2000), str(6))
        self.ser.write(string)

    def disable_altitudehold(self):
        string = 'Q%s;%s' % (str(1000), str(6))
        self.ser.write(string)

    def test_roll(self, thrust):
        if self.auto_switch > 1700:
            self.pitch = 1500
            self.roll = self.filter_thrust(thrust)
            self.send_receiver_commands()

    def test_pitch(self, thrust):
        if self.auto_switch > 1700:
            self.roll = 1500
            self.pitch = self.filter_thrust(thrust)
            self.send_receiver_commands()

    def test_roll_pitch(self, roll, pitch):
        if self.auto_switch > 1700:
            self.roll = self.filter_thrust(roll)
            self.pitch = self.filter_thrust(pitch)
            self.send_receiver_commands()

    def test_response(self):
        if self.auto_switch > 1700:
            if abs(time.time() - self.previous_time) > 2:
                self.direction = not self.direction
                self.previous_time = time.time()
            if self.direction:
                self.roll = 1550
            else:
                self.roll = 1450
            self.send_receiver_commands()

    def send_receiver_commands(self):
        string = '9%s;%s;%s;%s' % (str(self.roll), str(
            self.pitch), str(self.yaw), str(self.throttle))
        self.ser.write(string)

    def send_controll_commands(self):
        string = '9%s;%s;%s;%s' % (str(self.roll), str(
            self.pitch), str(self.yaw), str(self.throttle))
        self.ser.write(string)

    def send_throttle_command(self):
        string = 'Q%s' % str(self.throttle)
        self.ser.write(string)

    def pitch(self, thrust):
        channel = 1
        string = 'Q%s;%s' % (str(thrust), str(channel))
        self.ser.write(string)

    def roll(self, thrust):
        channel = 0
        string = 'Q%s;%s' % (str(thrust), str(channel))
        self.ser.write(string)

    def filter_thrust(self, thrust):
        try:
            if int(thrust) >= 1700 or int(thrust) <= 1300:
                return 1500
            else:
                return int(thrust)
        except:
            return 1500

    def filter_throttle(self, throttle):
        try:
            if int(throttle) >= 1850:
                return 1850
            elif int(throttle) <= 1300:
                return 1300
            else:
                return int(throttle)
        except:
            return 1500

    def general_filter(self, receiver_value):
        try:
            receiver_value = int(receiver_value)
            if 1000 <= receiver_value <= 2000:
                return receiver_value
            else:
                return 1500

        except:
            return 1500

    def debug_altitude_hold(self):
        return
        'throttle %d \
        alitude %f \
        set point %f ' % (
            self.throttle,
            self.state_estimate.getAltitude(),
            self.position_controller)
