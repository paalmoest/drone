import pickle
import pylab as pl
import numpy as np

s = 'data/nessheimTorsdag2/test_6'
acceleration = pickle.load(open('%s/acceleration.dump' % s))
attitude = pickle.load(open('%s/attitude.dump' % s))
control_commands = pickle.load(open('%s/control_commands.dump' % s))
marker = pickle.load(open('%s/marker_positions.dump' % s))
altitude = pickle.load(open('%s/altitude.dump' % s))
state_log = pickle.load(open('%s/state_log.dump' % s))
pid = pickle.load(open('%s/pid_log.dump' % s))

marker = pickle.load(open('%s/marker_positions.dump' % s))
roll_pid = pickle.load(open('%s/pid_log_roll.dump' % s))
pitch_pid = pickle.load(open('%s/pid_log_pitch.dump' % s))

ukf_state = pickle.load(open('%s/ukf_state.dump' % s))
try:
    pid_alt = pickle.load(open('%s/pid_log_altitudeHold.dump' % s))
    meta_alt = pickle.load(open('%s/meta_pid_alt.dump' % s))
    meta = pickle.load(open('%s/meta_pid.dump' % s))
except:
    pass
   # pid_alt = []

z_velocity = [i.z_velocity for i in acceleration]
x = [i.x for i in acceleration]
y = [i.y for i in acceleration]
z = [i.z for i in acceleration]

x_pos = [u.state[0] for u in ukf_state]
y_pos = [u.state[2] for u in ukf_state]
#x_pos = [u.state[0] for u in ukf_state]
angle_x = [i.roll for i in attitude]
angle_y = [i.pitch for i in attitude]
angle_z = [i.yaw for i in attitude]
#timestamps = [i.timestamp for i in attitude]

marker_x = [m.x if m is not None else np.ma.masked for m in marker]
marker_y = [m.y if m is not None else np.ma.masked for m in marker]


x = [p.timestamp for p in pid]
correction = [p.correction for p in pid]
p_correction = [p.P_corretion for p in pid]
i_correction = [p.I_corretion for p in pid]
d_correction = [p.D_corretion for p in pid]

x_alt = [p.timestamp for p in pid_alt]
correction_alt = [p.correction for p in pid_alt]
p_correction_alt = [p.P_corretion for p in pid_alt]
i_correction_alt = [p.I_corretion for p in pid_alt]
d_correction_alt = [p.D_corretion for p in pid_alt]
observations_alt = [p.observation for p in pid_alt]
error_alt = [p.error for p in pid_alt]
target_alt = [p.target for p in pid_alt]
thrust_alt = [p.thrust for p in pid_alt]

observations = [p.observation for p in pid]
error = [p.error for p in pid]
target = [p.target for p in pid]
thrust = [p.thrust for p in pid]
# print target
dts = []


for i in range(len(pid) - 1):
    dt = pid[i + 1].timestamp - pid[i].timestamp

xt = [p.timestamp for p in control_commands]

pid_alt_dt = []
pid_dt = []
for i in range(len(altitude) - 1):
    dt = altitude[i + 1].timestamp - altitude[i].timestamp
    dts.append(dt)

for i in range(len(pid_alt) - 1):
    dt = pid_alt[i + 1].timestamp - pid_alt[i].timestamp
    pid_alt_dt.append(dt)

for i in range(len(pid) - 1):
    dt = pid[i + 1].timestamp - pid[i].timestamp
    pid_dt.append(dt)

try:
    print sum(pid_alt_dt) / len(pid_alt)
    print sum(pid_dt) / len(pid_dt)
except:
    pass
# for i in range(33):
    # print pid[25+i+1].timestamp - pid[25+i].timestamp
 #   print pid[25+i].correction
 #   print pid[25+i].target - state_log[869+i].state[0]
    # print pid[25+i].correction


# print acceleration[869].timestamp
#correction = [p.correction for p in pid]
# 1384778572.37
# 1384778565.67
sonar = [a.sonar for a in altitude]
baro = [a.barometer for a in altitude]
camera = [a.camera for a in altitude]


roll = [u.roll for u in control_commands]
throttle = [u.throttle for u in control_commands]
pitch = [u.pitch for u in control_commands]
yaw = [u.yaw for u in control_commands]
battery = [u.battery for u in control_commands]
#throttle_log = [u.throttle_log for u in control_commands]

est_alt = [s.state[0] for s in state_log]
est_z_velocity = [s.state[1] for s in state_log]
for i in range(len(est_alt)):
    if est_alt[i] == 4.1464522053350228:
        print i
for i in range(len(pid) - 1):
   # print i
    # print pid[i].target
#    print pid[i].target - pid[i].altitude
    if pid[i + 1].correction > 50:
        print pid[i + 1].timestamp - pid[i].timestamp
        print pid[i + 1].error
        print pid[i + 1].correction
        print pid[i + 1].P_corretion
        print pid[i + 1].I_corretion
        print pid[i + 1].D_corretion


def plot_altitude():
    pl.figure('Altitude')
  #  b = pl.plot(baro, color="r")
    pl.plot(sonar, color="g", label="sonar raw")
    pl.plot(est_alt, color="b", label="estimated altitude")
   # c = pl.plot(camera, color="m")
 #s   pl.legend((b[0], s[0], e[0], c[0]), ('barometer', 'sonar', 'Kalman sonar', 'Camera'))
    #pl.plot(sonar, color="m")
  #  pl.plot(z_velocity, color="b")


def plot_battery():
    pl.figure('battery Voltage')
    pl.plot(battery)

def plot_correction():
    pl.figure()
    pl.plot(x, correction, color="b")
    pl.figure('correction')
    p = pl.plot(x, p_correction, color="r")
    i = pl.plot(x, i_correction, color="b")
    d = pl.plot(x, d_correction, color="g")
    pl.legend((p[0], i[0], d[0]), ('P %d' %
              meta.P, 'I %f' % meta.I, 'D %d' % meta.D))


def plot_correction_alt():
    pl.figure('Total correction')
    pl.plot(x_alt, correction_alt, color="b")
    pl.figure('Correction P, I, D ')
    p = pl.plot(x_alt, p_correction_alt, color="r")
    i = pl.plot(x_alt, i_correction_alt, color="b")
    d = pl.plot(x_alt, d_correction_alt, color="g")
    pl.legend((p[0], i[0], d[0]), ('P %d' %
              meta_alt.P, 'I %2f' % meta_alt.I, 'D %d' % meta_alt.D))
    pl.figure('thrust')
    pl.plot(x_alt, thrust_alt, color="b")




def plot_pid_alt():
    pl.figure('Target and observations')
    target = pl.plot(x_alt, target_alt, color="r")
    obs = pl.plot(x_alt, observations_alt, color="b")
    pl.legend((target[0], obs[0]), ('target', 'observations'))
 #   pl.figure()
   # pl.plot(error_alt)


def plot_pid():
    pl.figure('target and observations')
    t = pl.plot(x, target, color="r")
    obs = pl.plot(x, observations, color="b")
    pl.legend((t[0], obs[0]), ('target', 'observations'))
    pl.figure('error')
    pl.plot(error)

   # pl.figure()
   # pl.plot(xt, yaw)
    # 

def plot_attitude():
    pl.figure()
    #pl.ylim(1, 5)
    _x = pl.plot(angle_x, color="r")
    _y = pl.plot(angle_y, color="b")
    pl.legend((_x[0], _y[0]), ('roll', 'pitch'))
    pl.figure('heading')
    pl.plot(angle_z, color="g")

def plot_velocity():
    pl.figure('Vertical velocity')
    est_z = pl.plot(est_z_velocity, color="r")


def plot_accelration():
    pl.figure()
    pl.plot(x, color="r")
    pl.plot(y, color="m")
    pl.plot(z, color="b")


def plot_control():
    pl.figure()
    t = pl.plot(throttle, color="r")
  #  t_log = pl.plot(throttle_log, color="b")
    p = pl.plot(pitch, color="g")
    r = pl.plot(roll, color="y")
    y = pl.plot(yaw, color="y")
    pl.legend((r[0], p[0], y[0], t[0], t_log[0]),
             ('roll', 'pitch', 'yaw,', 'throttle', 'throttle log'))

def plot_marker():
    observationsX = [p.observation for p in roll_pid]
    observationsY = [p.observation for p in pitch_pid]
    pid_dt = [p.timestamp for p in roll_pid]

    pl.figure('marker pattern')
    pl.plot(observationsX, observationsY, marker='x')

    pl.figure('marker distance')
    pl.plot(pid_dt, observationsX, label='x', color="r")
    pl.plot(pid_dt, observationsY, label='Y', color="b")
    pl.legend(loc='upper left')


def plot_pid(pid, name):
    pid_dt = [p.timestamp for p in pid]
    correction = [p.correction for p in pid]
    p_correction = [p.P_corretion for p in pid]

    i_correction = [p.I_corretion for p in pid]
    d_correction = [p.D_corretion for p in pid]
    observations = [p.observation for p in pid]
    target = [p.target for p in pid]
    pl.figure('correction ' + name)
    pl.plot(pid_dt, correction, label="correction")
    pl.legend(loc='upper left')

    pl.figure('P, I, D correcitons ' + name)
    pl.plot(pid_dt, p_correction, label="P")
    pl.plot(pid_dt, i_correction, label="I")
    pl.plot(pid_dt, d_correction, label="D")
    pl.legend(loc='upper left')

    pl.figure('target and observations ' + name)
    pl.plot(pid_dt, target, label="Target")
    pl.plot(pid_dt, observations, label="Obsertions")
    pl.legend(loc='upper left')

def plot_throttle():
    pl.figure()
    t = pl.plot(throttle, color="r")
  #  t_log = pl.plot(throttle_log, color="b")
#plot_altitude()
#plot_marker()
plot_pid(pitch_pid, 'pitch')
plot_pid(roll_pid, 'roll')
#plot_battery()
#plot_correction_alt()
#plot_pid_alt()
#plot_battery()
#plot_pid()
# plot_correction()
plot_attitude()
# lot_control()
#print throttle[500:-1]
#plot_throttle()
#plot_velocity()
try:

    print meta_alt.P
    print meta_alt.I
    print meta_alt.D
except:
    pass
pl.show()

#pitch_cmd = pl.plot(_pitch_cmd, color='g')
#lines_filt = pl.scatter(len(c), c, color='r')
#lines_filt = pl.plot(c, color='r')
# pl.legend((roll[0], pitch[0], alt[0]),
#          ('roll angle', 'pitch angle', 'altitude'))
