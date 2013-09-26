import pickle
import matplotlib.pyplot as plt
from pid import PID
pid = PID(P=30, I=0.2, D=0, Derivator=0, Integrator=0, Integrator_max=100, Integrator_min=-100)
pid2 = PID(P=10, I=0.2, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
pid3 = PID(P=10, I=0.5, D=0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10)
#pid4 = PID(P=5, I=0.6, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
#pid5 = PID(P=5, I=0.6, D=0, Derivator=0, Integrator=0, Integrator_max=25, Integrator_min=-25)
pid.setPoint(1.50)
pid2.setPoint(1.50)
pid3.setPoint(1.50)




test = 'test_1'
roll = pickle.load(open('data/althold/%s/roll.dump' % test))
pitch = pickle.load(open('data/althold/%s/pitch.dump' % test))
yaw = pickle.load(open('data/althold/%s/yaw.dump' % test))
throttle = pickle.load(open('data/althold/%s/throttle.dump' % test))
baro = pickle.load(open('data/althold/%s/barometer.dump' % test))
sonar = pickle.load(open('data/althold/%s/sonar.dump' % test))

pid_array = []
pid_array2 = []
pid_array3 = []
for s in sonar:
	pid_array.append(pid.update(s))
	pid_array2.append(pid2.update(s))
	pid_array3.append(pid3.update(s))



plt.xlim(0, 1000)
plt.xlim(0,3)
plt.ylim(-100, 100)
plt.plot(throttle)
#plt.plot(throttle)
plt.plot(pid_array)
plt.plot(pid_array2)
plt.plot(pid_array3)
#plt.plot(baro)
#plt.plot(roll)
#plt.plot(yaw)
#plt.plot(pitch)
plt.ylabel('hoyde')
plt.show()

