
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from roboMain import Robot
import math
from random import randint


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

r.begin()
r.startBrush()
r.updateDeltaSpeedAngle(100, 0)

while True:
    dist = r.distance.getDistance()
    if math.isnan(dist):
        dist = 200
    if r.distance.getDistance() < 10:
        r.speed = 0
        r.updateSpeedAngle(70, randint(90, 270))
    if r.distance.getDistance() > 10:
        r.speed = 100

    time.sleep(0.01)

    key = getch()

    time.sleep(0.01)
