
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from roboMain import Robot
import numpy as np


r = Robot()

r.begin()
#r.startBrush()
#r.patternMove()

#r.turnToAngle(3.14/2)
r.pid_motors[0].setPoint(20)
r.pid_motors[1].setPoint(-20)
r.logDistance(1)

time.sleep(5)
r.stop()
print('End')