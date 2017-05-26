
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from roboMain import Robot
import numpy as np


B=np.array([0,100])
A=np.array([100,0])
D=np.array([0,-100])
C=np.array([-100,0])
r = Robot()

r.begin()

r.here_to_there(A,20)
r.here_to_there(B,20)
r.here_to_there(C,20)
r.here_to_there(D,20)
r.here_to_there(A,20)
r.here_to_there(B,20)
r.here_to_there(C,20)
r.here_to_there(D,20)


r.stop()
