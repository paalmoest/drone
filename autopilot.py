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


	def heading_hold(self):
		time.sleep(5)
		#then = datetime.datetime.now() + datetime.timedelta(seconds=5)
		hold_angle = False
		threshold = 0.20
		i = 0
		channel = 2
		thrust = 1500
		then = datetime.datetime.now()
		while True:
			if i % 3 == 0:
				print "im here waiting !"
				self.ser.write('s')
				s = self.ser.readline()
				data = s.split(',')
				print data
				print "oki"
				if len(data) > 10:
					print "YAW value: %s " % data[9]
					if not hold_angle:
						hold_angle = float(data[3])
						print "hold Angle set: ", hold_angle
						time.sleep(1)
					if (hold_angle > 6.13):
						pass
					heading = float(data[3])
					diff = hold_angle - heading
					print "diff : " , diff
					if then < datetime.datetime.now():
						if (diff <= -1 * threshold):
							thrust = 1400
							yaw_string = 'Q%s;%s' % (str(thrust), str(channel))
							self.ser.write(yaw_string)
							then = datetime.datetime.now() + datetime.timedelta(seconds=0.5)
							print "yaw left"
						elif (diff > threshold):
							thrust = 1600
							yaw_string = 'Q%s;%s' % (str(thrust), str(channel))
							self.ser.write(yaw_string)
							then = datetime.datetime.now() + datetime.timedelta(seconds=0.5)
							print "yaw right"
						else:
							print "HOLDING ! "
							thrust = 1500
							yaw_string = 'Q%s;%s' % (str(thrust), str(channel))
							self.ser.write(yaw_string)
							then = datetime.datetime.now() + datetime.timedelta(seconds=0.5)
			i += 1

	def test(self):
		time.sleep(10)
		then = datetime.datetime.now() + datetime.timedelta(seconds=5)
		thrust = 1150
		channel = 3
		i = 0
		while True:
			if then < datetime.datetime.now():
				#print "every 5sek"
				test_string = 'Q%s;%s' % (str(thrust), str(channel))
				print test_string
				if thrust <= 1300:
					thrust += 20
				self.ser.write('6')
				motor_values = self.ser.readline()
				print motor_values
				self.ser.write(test_string)
				#self.ser.write('6')
				#s = self.ser.readline()
				#print 'motorvalues: %s' % s
				then = datetime.datetime.now() + datetime.timedelta(seconds=5)

ap = AutoPilot()
ap.heading_hold()
#ap.test()
