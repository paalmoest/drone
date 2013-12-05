from main import Main
from pid import PID
heading_pid = PID(
    P=5,
    I=0,
    D=0,
    Derivator=0,
    Integrator=0,
    Integrator_max=25,
    Integrator_min=-25,
    maximum_thrust=25,
    minimum_thrust=-25,
)
cam_width = 160
cam_height = 90
drone = Main(
    host='10.0.0.44',
    port=5000,
    cam_width=cam_width,
    cam_height=cam_height,
    heading_pid=heading_pid
)
