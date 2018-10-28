
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
from mpi4py import MPI

comm = MPI.Comm

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

if comm.rank==1:
    r = wheel_control(controler_frequency=25)

    guidence = thread.start_new_thread(r.run_motor_control, (1,))
speed = np.array([0,0])*ureg.cm/ureg.seconds
if comm.rank==0:
    brushMotors = MC.motor_group([0x60, 0x60, 0X61], [1, 3, 4], [-1, 1, 1])

while True:
    key = getch()

    time.sleep(0.05)
    
    #r.setSpeedAngleManual()
    #worker=thread.start_new_thread(text.write,('Distance= '+str(distance),))
    #os.system('cls' if os.name == 'nt' else 'clear')
    #print(r.speed,r.RealAngle,r.temp)
   # print('angle=', angle[0],'distance= ',float(distance))
    if comm.rank==0:
        
        if key == 'q':
            speed*=0
            quit()
        elif key == 'w':
            speed+=5*ureg.cm/ureg.seconds
        elif key == 'd':
            speed[0]-=5*ureg.cm/ureg.seconds
            speed[1]+=5*ureg.cm/ureg.seconds
        elif key == 'a':
            speed[0]+=5*ureg.cm/ureg.seconds
            speed[1]-=5*ureg.cm/ureg.seconds
        elif key == 's':
            speed-=5*ureg.cm/ureg.seconds
        elif key == 'r':
            brushMotors.set_speed([255,255,255])

        elif key == 'e':
            brushMotors.set_speed([0,0,0])
        else:
            a = 1
        comm.send(speed,dest=1)
    if comm.rank==1:
        speed=comm.recv(source=0)    
        r.set_speed(speed)
