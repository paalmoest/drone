import serial
import time
import datetime
import time
import cv2
import cv
import csv
import matplotlib.pyplot as plt

#sample  1,0.02,-0.01,1.16,-0.19,0,1518,1497,1498,1590,1935,1969,0,0,1597,1587,1579,1577,0,0,0,0,10.20,1,


class AutoPilot():
	def __init__(self):
		self.init_serial()
		self.thrust_interval = 20

	def init_serial(self):
		self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
		self.ser.open()

	def run(self):
		time.sleep(10)  # timout for serial.
		#self.ser.write('arm_motors')
		i = 0
		j = 0
		then = datetime.datetime.now() + datetime.timedelta(seconds=20)
		while True:
			time.sleep(0.10)
			if i % 3 == 0:
				s = self.ser.readline()
				data = s.split(',')
				if len(data) > 1:
					bar_h = data[0]
					sonar_h = data[1]
					if bar_h:
						self.log_array.append(bar_h)
					if sonar_h:
						self.sonar_array.append(sonar_h)
	def validate_data(self, data):
		pass

	def hover(self):
		time.sleep(7)
		hover_height = 0.70
		in_flight = True
		self.ser.write('8')  # Get info
		self.ser.write('9')  # arm MOTORS!
		i = 0
		current_height = 0
		state = None
		throttle = 2000
		channel = 0
		while in_flight:
			print state
			print "sonar_hoyde :%s " % current_height
			if i % 3 == 0:
				s = self.ser.readline()
				data = s.split(',')
				state = data
				string = 'Q%s;%s' % (str(throttle), str(channel))
				self.ser.write(string)
	def get_copter_state(self,data):
		 return 'armed: %s heading %s hbar: %s hsonar:%s alltidehold: %s motor1: %s motor2:%s motor3: %s motor4: %s battery: %s flightmode: %s' % (data[0], data[3], data[4], data[5], data[6], data[15], data[16], data[17], data[18], data[23], data[24])
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
			print 'state; %s , radian-diff: %s' % (state , diff)
			if i % 3 == 0:
				s = self.ser.readline()
				data = s.split(',')
				if len(data) >= 24:
					copter_state = self.get_copter_state(data)
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
		
ap = AutoPilot()
ap.heading_hold()
#ap.hover()
#ap.test()
