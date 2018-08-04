
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


r = wheel_control()

guidence = thread.start_new_thread(r.run_motor_control, (1,))
speed = np.array([0,0])*ureg.cm/ureg.seconds
while True:
    key = getch()

    time.sleep(0.01)
    
    #r.setSpeedAngleManual()
    #worker=thread.start_new_thread(text.write,('Distance= '+str(distance),))
    os.system('cls' if os.name == 'nt' else 'clear')
    #print(r.speed,r.RealAngle,r.temp)
   # print('angle=', angle[0],'distance= ',float(distance))

    if key == 'q':
        speed*=0
        quit()
    elif key == 'w':
        speed+=5*ureg.cm/ureg.seconds
        # motors.set_speed([255,255])
    elif key == 'a':
        speed[0]-=5*ureg.cm/ureg.seconds
        speed[1]+=5*ureg.cm/ureg.seconds
    elif key == 'd':
        speed[0]+=5*ureg.cm/ureg.seconds
        speed[1]-=5*ureg.cm/ureg.seconds
    elif key == 's':
        speed-=5*ureg.cm/ureg.seconds
    elif key == 'r':
        pass

    elif key == 'e':
        pass
    else:
        a = 1
    r.set_speed(speed)
