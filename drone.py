from main import Main
from pid import PID
heading_pid = PID(
    P=5,
    I=0,
    D=0,
    maximum_thrust=25,
    minimum_thrust=-25,
)
altitude_pid = PID(
    P=25,
    I=0.5,
    D=50,
    maximum_thrust=80,
    minimum_thrust=-80,
    Integrator_max=100,
    Integrator_min=-100
)


roll_pid = PID(
    P=15,
    I=0,
    D=0,
    maximum_thrust=50,
    minimum_thrust=-50,
    Integrator_max=100,
    Integrator_min=-100,
)

pitch_pid = PID(
    P=15,
    I=0,
    D=0,
    maximum_thrust=50,
    minimum_thrust=-50,
    Integrator_max=100,
    Integrator_min=-100,
)
cam_width = 320
cam_height = 240
drone = Main(
    host='10.0.0.44',
    port=5000,
    cam_width=cam_width,
    cam_height=cam_height,
    heading_pid=heading_pid,
    altitude_pid=altitude_pid,
    pitch_pid=pitch_pid,
    roll_pid=roll_pid,
)
