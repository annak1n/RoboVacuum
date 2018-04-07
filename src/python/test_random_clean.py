
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
r.startBrush()
r.random_run()
#r.logDistance(1)
#
time.sleep(5)
#r.stopBrush()
r.stop()
import matplotlib.pyplot as plt

plt.imshow(r.map.map)
plt.show()
print('End')
