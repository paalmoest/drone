import serial
import time

ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
ser.open()
time.sleep(5)
ser.write('8')
i = 1300
first = time.time()
while True:
    response = ser.readline()
    i += 1
    #print i
    s = 'Q%d;%d;%d;%d' % (1500, 1500, 1500, i)
    ser.write(s)
    if i >= 2000:
        i = 1300
        dt = time.time() - first
        print dt
        exit()
    print response
