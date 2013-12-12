import serial
import time
import pickle
import pylab as pl
ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
ser.open()
time.sleep(1)
ser.write('8')
i = 1300
first = time.time()
baro = []
while time.time() < (first + (60*2)):
    response = ser.readline()
    data = response.split(',')
    print data
    if len(data) >= 5:
        baro.append(data[5])


pickle.dump(baro, open('fasttrack.dump', 'wb'))

pl.figure()
pl.plot(baro)
pl.show()
