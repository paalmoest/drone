import pickle
import pylab as pl

s = 'data/sonar/test_13'
acceleration = pickle.load(open('%s/acceleration.dump' % s))
attitude = pickle.load(open('%s/attitude.dump' % s))
marker = pickle.load(open('%s/marker_positions.dump' % s))
control_commands = pickle.load(open('%s/control_commands.dump' % s))
marker = pickle.load(open('%s/marker_positions.dump' % s))
altitude = pickle.load(open('%s/altitude.dump' % s))
state_log = pickle.load(open('%s/state_log.dump' % s))
print state_log
pid = pickle.load(open('%s/pid_log.dump' % s))
try:
    meta = pickle.load(open('%s/meta_pid.dump' % s))
    meta_alt = pickle.load(open('%s/meta_pid_alt.dump' % s))
    pid_alt = pickle.load(open('%s/pid_log_altitudeHold.dump' % s))
except:
    pid_alt = []

z_velocity = [i.z_velocity for i in acceleration]
x = [i.x for i in acceleration]
y = [i.y for i in acceleration]
z = [i.z for i in acceleration]

angle_x = [i.roll for i in attitude]
angle_y = [i.pitch for i in attitude]
angle_z = [i.yaw for i in attitude]
#timestamps = [i.timestamp for i in attitude]

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

for i in range(len(attitude) - 1):
    dt = attitude[i + 1].timestamp - attitude[i].timestamp
    dts.append(dt)

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
#throttle_log = [u.throttle_log for u in control_commands]

est_alt = [s.state[0] for s in state_log]

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
    pl.plot(baro, color="r")
    pl.plot(sonar, color="g")
    #pl.plot(sonar, color="m")
    #pl.plot(camera, color="m")
  #  pl.plot(z_velocity, color="b")
    #pl.plot(est_alt, color="g")


def plot_correction():
    pl.figure()
    pl.plot(x, correction, color="b")
    pl.figure('correction')
    p = pl.plot(x, p_correction, color="r")
    i = pl.plot(x, i_correction, color="b")
    d = pl.plot(x, d_correction, color="g")
    pl.legend((p[0], i[0], d[0]), ('P %d' %
              meta.P, 'I %d' % meta.I, 'D %d' % meta.D))


def plot_correction_alt():
    pl.figure('Total correction')
    pl.plot(x_alt, correction_alt, color="b")
    pl.figure('Correction P, I, D ')
    p = pl.plot(x_alt, p_correction_alt, color="r")
    i = pl.plot(x_alt, i_correction_alt, color="b")
    d = pl.plot(x_alt, d_correction_alt, color="g")
    pl.legend((p[0], i[0], d[0]), ('P %d' %
              meta_alt.P, 'I %d' % meta_alt.I, 'D %d' % meta_alt.D))


def plot_pid_alt():
    pl.figure('Target and observations')
    target = pl.plot(x_alt, target_alt, color="r")
    obs = pl.plot(x_alt, observations_alt, color="b")
    pl.legend((target[0], obs[0]), ('target', 'observations'))
 #   pl.figure()
   # pl.plot(error_alt)


def plot_pid():
    pl.figure()
    pl.plot(x, target, color="r")
    pl.plot(x, observations, color="b")
    pl.figure()
    pl.plot(error)

   # pl.figure()
   # pl.plot(xt, yaw)
    # pl.figure()
    #pl.plot(x, thrust, color="b")


def plot_attitude():
    pl.figure()
    #pl.ylim(1, 5)
    #pl.plot(angle_x, color="r")
    #pl.plot(angle_y, color="b")
    pl.plot(angle_z, color="g")


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


def plot_throttle():
    pl.figure()
    t = pl.plot(throttle, color="r")
  #  t_log = pl.plot(throttle_log, color="b")

plot_altitude()
# plot_correction()
plot_correction_alt()
plot_pid_alt()
# plot_correction()
# plot_attitude()
# lot_control()
plot_throttle()
try:

    print meta.P
    print meta.I
    print meta.D
except:
    pass
pl.show()

#pitch_cmd = pl.plot(_pitch_cmd, color='g')
#lines_filt = pl.scatter(len(c), c, color='r')
#lines_filt = pl.plot(c, color='r')
# pl.legend((roll[0], pitch[0], alt[0]),
#          ('roll angle', 'pitch angle', 'altitude'))