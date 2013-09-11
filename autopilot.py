import serial
import time
import datetime


#sample  1,0.02,-0.01,1.16,-0.19,0,1518,1497,1498,1590,1935,1969,0,0,1597,1587,1579,1577,0,0,0,0,10.20,1,


class AutoPilot():
	def __init__(self, **kwargs):
		simulate = kwargs.get('simulate', False)
		if not simulate:
			self.connect_to_drone()
		self.thrust_limit = 1700
		self.thrust_step = kwargs.get('thrust_step', 20)
		self.pixel_threshold = kwargs.get('pixel_threshold', 50)
		self.time_interval = kwargs.get('time_interval', 0.1) 
		self.cam_width = kwargs.get('cam_width')
		self.cam_height = kwargs.get('cam_height')
		self.vebrose = kwargs.get('vebrose')
		self.cam_center = [self.cam_width / 2, self.cam_height / 2]
		self.auto_switch = False
		self.roll = 1500
		self.pitch = 1500
		self.yaw = 1500
		self.throttle = 1300
		self.alitudehold = False

		self.roll_thrust = 1500
		self.pitch_thrust = 1500
		self.init_thrust = 1500
		self.then = datetime.datetime.now()
		

	def connect_to_drone(self):
		self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
		self.ser.open()
		string = 'connect_to_drone .'
		wait = 5
		for x in range(wait):
			string += '.'
			print string
			time.sleep(1)

		self.ser.write('8')

	def run(self):
		sensor_data = self.read_sensors()
		if sensor_data:
			self.set_state(sensor_data)
		#print self.get_copter_state(sensor_data)

	def read_sensors(self):
		s = self.ser.readline()
		sensor_data = s.split(',')
		if len(sensor_data) >= 24:
			return sensor_data
		else:
			return None

	def enable_alitude_hold(self):
		string = 'Q%s;%s' % (str(2000), str(6))
		self.ser.write(string)

	def position_hold(self, pos_x, pos_y):
		if self.auto_switch > 1700:
			if not self.alitudehold:
				self.enable_alitude_hold()
				self.alitudehold = True
			if abs(self.cam_center[0] - pos_x) <= self.pixel_threshold:
				self.roll = self.roll_thrust
			else:
				x_diff = self.cam_center[0] - pos_x
				if x_diff > 0:
					self.roll = self.roll_thrust - self.thrust_step
				else:
					self.roll = self.roll_thrust + self.thrust_step

			if abs(self.cam_center[1] - pos_y) <= self.pixel_threshold:
				self.pitch = self.pitch_thrust
			else:
				y_diff = self.cam_center[1] - pos_y
				if y_diff > 0:
					self.pitch = self.pitch_thrust + self.thrust_step
				else:
					self.pitch = self.pitch_thrust - self.thrust_step
			self.send_receiver_commands()

	def position_hold_weighted(self, pos_x, pos_y):
		pass

	def simulate_position_hold(self, pos_x, pos_y):
		if pos_x:
			if abs(self.cam_center[0] - pos_x) <= self.pixel_threshold:
				self.roll = self.roll_thrust
			else:
				x_diff = self.cam_center[0] - pos_x
				if x_diff > 0:
					self.roll = self.roll_thrust - self.thrust_step
				else:
					self.roll = self.roll_thrust + self.thrust_step

			if abs(self.cam_center[1] - pos_y) <= self.pixel_threshold:
				self.pitch = self.pitch_thrust
			else:
				y_diff = self.cam_center[1] - pos_y
				if y_diff > 0:
					self.pitch = self.pitch_thrust + self.thrust_step
				else:
					self.pitch = self.pitch_thrust - self.thrust_step
		else:
			self.roll = 1500
			self.pitch = 1500

		return self.roll, self.pitch

	def send_receiver_commands(self):
 		string = '9%s;%s;%s;%s' % (str(self.roll), str(self.pitch), str(self.yaw), str(self.throttle))
		self.ser.write(string)

	def pitch(self, thrust):
		channel = 1
		string = 'Q%s;%s' % (str(thrust), str(channel))
		self.ser.write(string)

	def roll(self, thrust):
		channel = 0
		string = 'Q%s;%s' % (str(thrust), str(channel))
		self.ser.write(string)

	def get_state(self, data):
		return 'armed: %s heading %s hbar: %s hsonar:%s alltidehold: %s motor1: %s motor2:%s motor3: %s motor4: %s battery: %s flightmode: %s' % (data[0], data[3], data[4], data[5], data[6], data[15], data[16], data[17], data[18], data[23], data[24])

	#def get_state(self):
	#	return 'armed: %s heading %s hbar: %s hsonar:%s alltidehold: %s motor1: %s motor2:%s motor3: %s motor4: %s battery: %s flightmode: %s' % (data[0], data[3], data[4], data[5], data[6], data[15], data[16], data[17], data[18], data[23], data[24])
	
	def filter_thrust(self, thrust):
		try:
			if int(thrust) >= 1700 or int(thrust) <= 1400:
				return 1500
			else:
				return int(thrust)
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

	def pp_receiver_commands(self):
		return 'roll: %d pitch: %d yaw: %d  throttle: %d auto: %d' % (self.roll, self.pitch, self.yaw, self.throttle, self.auto_switch)

	def set_state(self, data):
		#drone_state = data.split(',')
		if len(data) >= 24:
			self.roll = self.filter_thrust(data[7])
			self.pitch = self.filter_thrust(data[8])
			self.yaw = self.filter_thrust(data[9])
			self.throttle = self.filter_thrust(data[10])
			self.auto_switch = self.general_filter(data[14])
			self.height_barometer = data[4]
			self.height_sonar = data[5]
			self.armed = data[0]
			self.angle_x = data[1]
			self.angle_y = data[2]
			self.heading = data[3]


	def heading_hold(self):
		time.sleep(5)
		#then = datetime.datetime.now() + datetime.timedelta(seconds=5)
		hold_angle = False
		threshold = 0.20
		yaw_interval = 100
		i = 0
		channel = 2
		then = datetime.datetime.now()
		state = ""
		yaw_level = 1500
		left_yaw = yaw_level - yaw_interval

		right_yaw = yaw_level + yaw_interval
		receiver_autopilot = 1500
		self.ser.write('8')
		copter_state = ""
		diff = 0
		time_interval = 0.5
		while True:
			print copter_state
			print 'state; %s ,radian-diff: %s' % (state, diff)
			if i % 3 == 0:
				s = self.ser.readline()
				data = s.split(',')
				if len(data) >= 24:
					copter_state = self.get_state(data)
					if not hold_angle:
						hold_angle = float(data[3])
						print "hold Angle set: ", hold_angle
						time.sleep(1)
					heading = float(data[3])
					diff = hold_angle - heading
					try:
						receiver_autopilot = int(data[14])
					except:
						pass
					print "auto: %s  " % receiver_autopilot
					if receiver_autopilot > 1500:
						if then < datetime.datetime.now():
							if (diff <= -1 * threshold):
								yaw_string = 'Q%s;%s' % (str(left_yaw), str(channel))
								self.ser.write(yaw_string)
								then = datetime.datetime.now() + datetime.timedelta(seconds=time_interval)
								state = "yaw left"
							elif (diff > threshold):
								yaw_string = 'Q%s;%s' % (str(right_yaw), str(channel))
								self.ser.write(yaw_string)
								then = datetime.datetime.now() + datetime.timedelta(seconds=time_interval)
								state = "yaw right"
							else:
								state = "HOLDING ! "
								yaw_string = 'Q%s;%s' % (str(yaw_level), str(channel))
								self.ser.write(yaw_string)
								then = datetime.datetime.now() + datetime.timedelta(seconds=time_interval)
			i += 1

	def test(self):
		time.sleep(5)
		i = 0
		self.ser.write('8')
		while True:
			if i % 3 == 0:
				s = self.ser.readline()
				data = s.split(',')
				print self.get_copter_state(data)

#ap = AutoPilot()
#ap.heading_hold()
#ap.hover()
#ap.test()
