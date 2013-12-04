import serial
import time
import pickle
import os
import numpy as np
# sample
# 1,0.02,-0.01,1.16,-0.19,0,1518,1497,1498,1590,1935,1969,0,0,1597,1587,1579,1577,0,0,0,0,10.20,1,


class StateLog():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.state = kwargs.get('state', None)


class ControlCommands():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.roll = kwargs.get('roll', None)
        self.pitch = kwargs.get('pitch', None)
        self.yaw = kwargs.get('yaw', None)
        self.throttle = kwargs.get('throttle', None)
        self.throttle_log = kwargs.get('throttle_log', None)


class Altitude():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.barometer = kwargs.get('barometer', None)
        self.camera = kwargs.get('camera', None)
        self.sonar = kwargs.get('sonar', None)
        self.z_velocity = kwargs.get('z_velocity', None)


class Acceleration():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.x = kwargs.get('x', None)
        self.y = kwargs.get('y', None)
        self.z = kwargs.get('z', None)
        self.z_velocity = kwargs.get('z_velocity', None)


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

    def __init__(self,  state_estimate, **kwargs):
        simulate = kwargs.get('simulate', False)
        if not simulate:
            self.connect_to_drone()
        self.state_estimate = state_estimate
        self.pixel_threshold = kwargs.get('pixel_threshold', 100)
        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.vebrose = kwargs.get('vebrose')
        self.cam_center = [self.cam_width / 2, self.cam_height / 2]
        self.marker = False
        self.auto_switch = False
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = 1500
        self.altitude_sonar = 0.00
        self.altitude_barometer = 0.00
        self._altitudehold = ''
        self.z_velocity = 0.0
        self.mode = ''
        self.aux1 = ''
        self.aux2 = ''
        self.accel_raw_x = 0.0
        self.accel_raw_y = 0.0
        self.accel_raw_z = 0.0
        self.angle_x = 0.0
        self.altitude_camera = 0.0
        self.left = True
        self.previous_time = time.time()

        self.roll_thrust = 1500
        self.pitch_thrust = 1500
        self.init_thrust = 1500
        self.direction = True

        self.init_logging()

    def init_logging(self):
        self.control_commands = []
        self.sonar_array = []
        self.baro_array = []
        self.state_log = []
        self.pid_log = []
        self.maker_positions = []
        self.acceleration = []
        self.attitude = []
        self.altitude = []

    def log(self):
        self.state_log.append(
            StateLog(state=self.state_estimate.state)
        )
        self.control_commands.append(
            ControlCommands(
                roll=self.roll,
                pitch=self.pitch,
                yaw=self.yaw,
                throttle=self.throttle,
                throttle_log=self.throttle_log
            )
        )
        self.altitude.append(
            Altitude(
                barometer=self.altitude_barometer,
                camera=self.altitude_camera,
                sonar=self.altitude_sonar,
                z_velocity=self.z_velocity
            )
        )
        self.attitude.append(
            Attitude(
                roll=self.angle_x,
                pitch=self.angle_y,
                yaw=self.heading
            )
        )
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
        pickle.dump(self.control_commands, open(
            'data/%s/%s.dump' % (mypath, 'control_commands'), 'wb'))
        pickle.dump(self.acceleration, open(
            'data/%s/%s.dump' % (mypath, 'acceleration'), 'wb'))
        pickle.dump(self.pid_log, open(
            'data/%s/%s.dump' % (mypath, 'pid_log'), 'wb'))
        pickle.dump(self.state_log, open(
            'data/%s/%s.dump' % (mypath, 'state_log'), 'wb'))
        pickle.dump(
            self.attitude,
            open('data/%s/%s.dump' % (mypath, 'attitude'), 'wb'))
        pickle.dump(
            self.altitude,
            open('data/%s/%s.dump' % (mypath, 'altitude'), 'wb'))
        pickle.dump(self.maker_positions, open(
            'data/%s/%s.dump' % (mypath, 'marker_positions'), 'wb'))
        pickle.dump(self.meta_pid, open(
            'data/%s/%s.dump' % (mypath, 'meta_pid'), 'wb'))
        exit()

    def connect_to_drone(self):
        self.ser = serial.Serial(
            port='/dev/ttyACM0', baudrate=115200, timeout=0.01)
        self.ser.open()
        string = 'connect_to_drone .'
        wait = 5
        for x in range(wait):
            string += '.'
            print string
            time.sleep(1)

        self.ser.write('8')

    def disconnect_from_drone(self):
        self.ser.write('x')
        self.ser.close()

    def update_marker(self, marker):
        if marker:
            self.altitude_camera = marker.get_altitude()
            self.marker = marker
        else:
            self.marker = False
        self.maker_positions.append(marker)

    def _read_sensors(self):
        while True:
            s = self.ser.readline()
            if not self.ser.inWaiting():
                break
        sensor_data = s.split(',')
        self.update_state(sensor_data)

    def update_state(self, data):
        try:
            self.roll = int(data[0])
            self.pitch = int(data[1])
            self.yaw = int(data[2])
            self.throttle = int(data[3])
            self.auto_switch = int(data[4])
            self.altitude_barometer = float(data[5])
            self.state_estimate.update(np.array([self.altitude_barometer]))
        except:
            pass
       # self.log()

    def print_commands(self):
        return 'roll %d pitch %d yaw %d throttle %d auto %d marker %f' % (
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
            self.auto_switch,
            self.altitude_camera,
        )

    def print_alt_hold(self):
        return 'throttle %d auto %d altitude %f' % (
            self.throttle,
            self.auto_switch,
            self.altitude_barometer,
        )

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
            self.throttle = 2000

    def send_receiver_commands(self):
        string = '9%s;%s;%s;%s' % (str(self.roll), str(
            self.pitch), str(self.yaw), str(self.throttle))
        self.ser.write(string)

    def send_control_commands(self):
        string = 'Q%d;%d;%d;%d' % self.roll,self.pitch, self.yaw, self.throttle)
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
