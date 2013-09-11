import time
import serial


class GPS:
	def connect_to_drone(self):
		self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
		self.ser.open()
		string = 'connect_to_drone .'
		wait = 5
		for x in range(wait):
			string += '.'
			print string
			time.sleep(1)

		self.ser.write('y')

	def main(self):
		self.connect_to_drone()
		i = 0
		while True:
			if i % 3 == 0:
				print self.ser.readline()

gps = GPS()
gps.main()
