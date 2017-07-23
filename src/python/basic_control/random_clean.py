
import time
import os
# brush.set_speed([255,255])
import tty
import sys
import select
from roboMain import Robot
import math
from random import randint





r = Robot()

#r.begin()
r.startBrush()
r.updateDeltaSpeedAngle(100, 0)

while True:
    dist = r.distance.getDistance()
    print(dist, r.RealAngle, r.speed)
    if math.isnan(dist):
        dist = 200
    
    if r.distance.getDistance() < 15:
        r.speed = 0
        r.updateSpeedAngle(70, randint(90, 270))
    if r.distance.getDistance() > 15:
        r.speed = 100
    r.setSpeedAngleManual()
    time.sleep(0.01)

