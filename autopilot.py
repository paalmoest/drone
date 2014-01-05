import serial
import time
import pickle
import os
import numpy as np
from position_estimator import LinearPosition


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
        self.battery = kwargs.get('battery', None)


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


class MarkerSync():

    def __init__(self, **kwargs):
        self.timestamp = time.time()
        self.x = kwargs.get('x', np.ma.masked)
        self.y = kwargs.get('y', np.ma.masked)


class Attitude():

    def __init__(self, **kwargs):
        self.roll = kwargs.get('roll', None)
        self.pitch = kwargs.get('pitch', None)
        self.yaw = kwargs.get('yaw', None)
        self.timestamp = time.time()


class AutoPilot():

    def __init__(self, state_estimate, **kwargs):
        simulate = kwargs.get('simulate', False)
        if not simulate:
            self.connect_to_drone()
        self.state_estimate = state_estimate
        self.linear_position = LinearPosition()
        self.cam_width = kwargs.get('cam_width', 320)
        self.cam_height = kwargs.get('cam_height', 240)
        self.marker = False
        self.auto_switch = False
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.throttle = 1500
        self.mode = 1000
        self.last_roll = 0
        self.last_pitch = 0
        self.altitude_sonar = 0.00
        self.altitude_barometer = 0.00
        self.z_velocity = 0.0
        self.accel_raw_x = 0.0
        self.accel_raw_y = 0.0
        self.accel_raw_z = 0.0
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.heading = 0.0
        self.battery = 0.0
        self.altitude_camera = 0.0

        self.x_distance_to_marker = np.ma.masked
        self.y_distance_to_marker = np.ma.masked
        self.previous_time = time.time()
        self.init_logging()

    def init_logging(self):
        self.control_commands = []
        self.sonar_array = []
        self.baro_array = []
        self.state_log = []
        self.pid_log = []
        self.pid_log_altitudeHold = []
        self.pid_log_roll = []
        self.pid_log_pitch = []
        self.maker_positions = []
        self.acceleration = []
        self.attitude = []
        self.altitude = []
        self.state_log_marker = []
        self.marker_sync = []
        self.ukf_state = []

    def log_control_commands(self):
        self.control_commands.append(
            ControlCommands(
                roll=self.roll,
                pitch=self.pitch,
                yaw=self.yaw,
                throttle=self.throttle,
                battery=self.battery,
            )
        )

    def log_marker(self):
        self.marker_sync.append(
            MarkerSync(
                x=self.x_distance_to_marker, y=self.y_distance_to_marker)
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

    def log_ukf(self, state):
        self.ukf_state.append(
            StateLog(state=state)
        )

    def log(self):
        # self.log_acceleration()
       # self.log_state()
        self.log_control_commands()
        self.log_attitude()
        self.log_altitude()
        # self.log_acceleration()
        self.log_marker()

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
        pickle.dump(self.pid_log_roll, open(
            'data/%s/%s.dump' % (mypath, 'pid_log_roll'), 'wb'))
        pickle.dump(self.pid_log_pitch, open(
            'data/%s/%s.dump' % (mypath, 'pid_log_pitch'), 'wb'))
        pickle.dump(self.state_log_marker, open(
            'data/%s/%s.dump' % (mypath, 'state_log_marker'), 'wb'))
        pickle.dump(self.ukf_state, open(
            'data/%s/%s.dump' % (mypath, 'ukf_state'), 'wb'))
        try:
            pickle.dump(self.meta_pid_alt, open(
                'data/%s/%s.dump' % (mypath, 'meta_pid_alt'), 'wb'))
        except:
            pass
        try:
            pickle.dump(self.meta_pid, open(
                'data/%s/%s.dump' % (mypath, 'meta_pid'), 'wb'))
        except:
            pass
        exit()

    def connect_to_drone(self):
        self.ser = serial.Serial(
            port='/dev/ttyACM0', baudrate=115200, timeout=0.02)
        self.ser.open()
        string = 'connect_to_drone .'
        wait = 3
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

    def update_linearKf(self):
        self.calcualteMarkerDistance()
        observations = [
            self.x_distance_to_marker,
            self.y_distance_to_marker
        ]
        self.linear_position.update(observations)

    def update_state(self, data):
        try:
            self.throttle = int(data[0])
            self.mode = float(data[1])
            self.auto_switch = int(data[2])
            self.angle_x = float(data[6])
            self.angle_y = float(data[7])
            self.heading = float(data[8])
            self.altitude_sonar = float(data[9])
            self.battery = float(data[10])
            self.state_estimate.update(np.array([self.altitude_sonar]))
        except:
            pass
        self.update_linearKf()
        # self.log()

    def update_state_legacy(self, data):
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
            self.battery = float(data[10])
            self.mode = float(data[11])
            self.state_estimate.update(np.array([self.altitude_sonar]))
        except:
            pass
        self.log()

    def update_marker(self, marker):
        if marker:
            self.altitude_camera = marker.get_altitude()
            self.marker = marker
            #self.state_estimate_marker.update([marker.x, marker.y])
        else:
            self.marker = False
            #self.state_estimate_marker.update([np.ma.masked, np.ma.masked])
        # self.maker_positions.append(marker)

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
        # string = 'Q%d;%d;%d;%d;' % (
        #    self.roll, self.pitch, self.yaw, self.throttle)
        #string = 'Q%d;' % (self.throttle)
        string = '9%d;%d;%d;%d;' % (
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle)
        self.ser.write(string)

    def send_throttle_command(self):
        string = 'Q%s;' % str(self.throttle)
        self.ser.write(string)

    def calcualteMarkerDistance(self):
        camera_x_center = self.cam_width / 2
        camera_y_center = self.cam_height / 2
        z = self.state_estimate.getAltitude() + 0.10
        lx = np.sin(self.angle_x) * z
        ly = np.sin(self.angle_y) * z
       # pixels_per_meter = (121.742 / z)
        pixels_per_meter = (248 / z)
        #pixels_per_meter = (200 / z)
        if self.marker:
            x_diff_pixels = self.cx - camera_x_center
            y_diff_pixels = self.cy - camera_y_center
            y = (y_diff_pixels / pixels_per_meter)
            x = (x_diff_pixels / pixels_per_meter)
            mx = x - lx
            my = ly - y
            self.x_distance_to_marker = mx
            self.y_distance_to_marker = my
            # print 'x_marker: %.2f y_marker: %.2f' % (mx, my)
        else:
            # print 'angle: %.2f' % (self.angle_x)
            self.y_distance_to_marker = np.ma.masked
            self.x_distance_to_marker = np.ma.masked

    def getControlCommand(self):

        c1 = 0.001
        roll = (self.roll - 1500) / 500
        pitch = (self.pitch - 1500) / 500

        u_pitch = c1 * (pitch - self.last_pitch)
        u_roll = c1 * (roll - self.last_roll)
        self.last_roll = roll
        self.last_pitch = pitch
        return np.asarray([0, 0, 0, 0, 0, 0])
