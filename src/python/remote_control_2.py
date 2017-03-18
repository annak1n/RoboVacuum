
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from roboMain import Robot


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


r = Robot()


while True:
    key = getch()

    time.sleep(0.01)
    print(r.speed,r.angle)
    #worker=thread.start_new_thread(text.write,('Distance= '+str(distance),))
    os.system('cls' if os.name == 'nt' else 'clear')
   # print('angle=', angle[0],'distance= ',float(distance))
    if key == 'q':
        r.updateSpeedAngle(0, 0)
        quit()
    elif key == 'w':
        r.updateDeltaSpeedAngle(10, 0)
        # motors.set_speed([255,255])
    elif key == 'a':
        r.updateDeltaSpeedAngle(0, 5)
        # motors.set_speed([100,-100])
    elif key == 'd':
        r.updateDeltaSpeedAngle(0, -5)
        # motors.set_speed([-100,100])
    elif key == 's':
        r.updateDeltaSpeedAngle(-10, 0)
    elif key == 'r':
        r.startBrush()

    elif key == 'e':
        r.stopBrush()
        r.updateSpeedAngle(0, 0)
    else:
        a = 1
