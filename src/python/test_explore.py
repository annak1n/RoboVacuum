
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
r.patternMove()


r.stop()
