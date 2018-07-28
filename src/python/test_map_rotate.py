
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
#r.pid_motors[0].setPoint(20*r.ureg.cm/r.ureg.second)
#r.pid_motors[1].setPoint(-20*r.ureg.cm/r.ureg.second)
#r.logDistance(1)
#r.startBrush()
r.set_velocities(10*r.ureg.cm/r.ureg.second,0*r.ureg.radian/r.ureg.second)
time.sleep(5)
r.set_velocities(0*r.ureg.cm/r.ureg.second,0.25*r.ureg.radian/r.ureg.second)
#r.stopBrush()
time.sleep(5)
r.stop()
r.build_map()
print('End')
