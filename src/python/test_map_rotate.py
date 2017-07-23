
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from roboMain import Robot
import numpy as np


r = Robot()

r.papirus.update()

r.begin()
#r.startBrush()
#r.patternMove()

#r.turnToAngle(3.14/2)
r.pid_motors[0].setPoint(10)
r.pid_motors[1].setPoint(-10)
r.logDistance(1)

#self.controlerWS[0]=1
#.controlerWS[1]=-1
time.sleep(5)

r.stop()
r.papirus.update()
print('End')
