import pickle
import matplotlib.pyplot as plt
from pid import PID
import time
pid = PID(P=0.6, I=0, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
#pid2 = PID(P=10, I=0.2, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
#pid3 = PID(P=10, I=0.5, D=0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10)
#pid4 = PID(P=5, I=0.6, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
#pid5 = PID(P=5, I=0.6, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
pid.setPoint(2)
#pid2.setPoint(1.50)
#pid3.setPoint(1.50)




test = 'test_15'
baro = pickle.load(open('data/hovedbygget/%s/barometer.dump' % test))

prev = time.time()
throttle = 1700
for b in baro:
	if time.time() > prev:
		throttle += pid.update(b)
		print "throttle: %d alt: %f"  % (throttle, b)
		time.sleep(0.15)


#plt.xlim(0, 1000)
#plt.xlim(0, 3)
#plt.ylim(-100, 100)
#plt.plot(throttle)
#plt.plot(throttle)
#plt.plot(pid_array)
#plt.plot(pid_array2)
#lt.plot(pid_array3)
#plt.plot(baro)
#plt.plot(roll)
#plt.plot(yaw)
#plt.plot(pitch)
#plt.ylabel('hoyde')
#plt.show()

