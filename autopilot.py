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
        self.marker = False
        self.auto_switch = False
        self.set_point_switch = False
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = 1500
        self.altitude_sonar = 0.00
        self.altitude_barometer = 0.00
        self.z_velocity = 0.0
        self.accel_raw_x = 0.0
        self.accel_raw_y = 0.0
        self.accel_raw_z = 0.0
        self.angle_x = 0.0
        self.heading = 0.0
        self.altitude_camera = 0.0
        self.left = True
        self.previous_time = time.time()
        self.init_logging()

    def init_logging(self):
        self.control_commands = []
        self.sonar_array = []
        self.baro_array = []
        self.state_log = []
        self.pid_log = []
        self.pid_log_altitudeHold = []
        self.maker_positions = []
        self.acceleration = []
        self.attitude = []
        self.altitude = []

    def log_control_commands(self):
        self.control_commands.append(
            ControlCommands(
                roll=self.roll,
                pitch=self.pitch,
                yaw=self.yaw,
                throttle=self.throttle,
            )
        )

    def log_altitude(self):
        self.altitude.append(
            Altitude(
                barometer=self.altitude_barometer,
                camera=self.altitude_camera,
                sonar=self.altitude_sonar,
                z_velocity=self.z_velocity
            )
        )

    def log_attitude(self):
        self.attitude.append(
            Attitude(
                roll=self.angle_x,
                pitch=self.angle_y,
                yaw=self.heading
            )
        )

    def log_acceleration(self):
        self.acceleration.append(
            Acceleration(
                x=self.accel_raw_x,
                y=self.accel_raw_y,
                z=self.accel_raw_z,
                z_velocity=self.z_velocity
            )
        )

    def log_state(self):
        self.state_log.append(
            StateLog(state=self.state_estimate.state)
        )

    def log(self):
        #self.log_state()
        #self.log_acceleration()
        self.log_control_commands()
        self.log_attitude()
        #self.log_acceleration()
        self.log_altitude()

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
        pickle.dump(self.pid_log_altitudeHold, open(
            'data/%s/%s.dump' % (mypath, 'pid_log_altitudeHold'), 'wb'))
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
        try:
            pickle.dump(self.meta_pid, open(
                'data/%s/%s.dump' % (mypath, 'meta_pid'), 'wb'))
            pickle.dump(self.meta_pid_alt, open(
                'data/%s/%s.dump' % (mypath, 'meta_pid_alt'), 'wb'))
        except:
            pass
        exit()

    def connect_to_drone(self):
        self.ser = serial.Serial(
            port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
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

    def read_sensors(self):
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
            self.angle_x = float(data[6])
            self.angle_y = float(data[7])
            self.heading = float(data[8])
            self.altitude_sonar = float(data[9])
           # self.state_estimate.update(np.array([self.altitude_barometer]))
            self.state_estimate.update(np.array([self.altitude_sonar]))
        except:
            pass
        self.log()

    def update_marker(self, marker):
        if marker:
            self.altitude_camera = marker.get_altitude()
            self.marker = marker
        else:
            self.marker = False
        self.maker_positions.append(marker)

    def print_commands(self):
        return 'roll %d pitch %d yaw %d throttle %d auto %d marker %f sonar %f baro %f' % (
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
            self.auto_switch,
            self.altitude_camera,
            self.altitude_sonar,
            self.altitude_barometer,
        )

    def print_alt_hold(self):
        return 'throttle %d auto %d altitude %f' % (
            self.throttle,
            self.auto_switch,
            self.altitude_barometer,
        )

    def send_control_commands(self):
        string = 'Q%d;%d;%d;%d;' % (
            self.roll, self.pitch, self.yaw, self.throttle)
        self.ser.write(string)

    def send_throttle_command(self):
        string = 'Q%s' % str(self.throttle)
        self.ser.write(string)
