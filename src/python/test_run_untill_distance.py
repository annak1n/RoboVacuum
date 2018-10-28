
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from DWR_wheel_control import wheel_control
from dimensions import ureg
import thread
import numpy as np
import motor_control.motor_class as MC
from ADC.IR_distance import distanceMeas
import math

def getch():
    try:
        import termios
    except ImportError:
        # Non-POSIX. Return msvcrt's (Windows') getch.
        import msvcrt
        return msvcrt.getch

    # POSIX system. Create and return a getch that manipulates the tty.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

dm = distanceMeas(calibrationFile='calibration_data.db')
r = wheel_control(controler_frequency=10/ureg.seconds)

guidence = thread.start_new_thread(r.run_motor_control, (1,))
speed = np.array([100,100])*ureg.cm/ureg.seconds

brushMotors = MC.motor_group([0x60, 0x60, 0X61], [1, 3, 4], [-1, 1, 1])
r.set_speed(speed)

distance=120

while distance > 30:
    distance = dm.getDistance()
    if math.isnan(distance):
      distance = 120
    print(distance)
    time.sleep(0.01)

speed*=0
r.set_speed(speed)
