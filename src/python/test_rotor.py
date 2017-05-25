from rotary_encoder import wheel_encoder as encoder
import motor_control.motor_class as MC
import time
from control.pid import PID
from math import pi, copysign

from accelerometer.BNO055 import BNO055

import wiringpi



def calc_speed(clicks,dt):
  return(((float(clicks)/1024.0)*2.0*pi*3.0)/dt)


driveMotors = MC.motor_group([0x61, 0x61], [1, 3], [1, -1])

bno = BNO055(serial_port='/dev/ttyUSB0', rst=None)
bno.begin()
###this is just playing around with PID and motor controler
enc=encoder.WheelEncoder()
print(enc.read_counters())
#driveMotors.set_speed([127, 0])
t1=time.time()
t2=t1
total_clicks=2**12
speed=00.0
clicks=0;
P=0.5
I=1
D=0
pid_left=PID(P=P,I=I,D=D)
pid_right=PID(P=P,I=I,D=D) 
pid_left.setPoint(speed)
pid_right.setPoint(speed)
dt=0.01
s_left=+0
s_right=+0

pid_angle=PID(P=10,I=0,D=2,Angle=True)
setangle=-90
pid_angle.setPoint(setangle)
while clicks<total_clicks: 
  t1=time.clock()
  c1,c2=enc.read_counters()
  RealAngle = bno.read_euler()[0]
  speed_left=copysign(calc_speed(c2,dt),s_left)
  speed_right=copysign(calc_speed(c1,dt),s_right)
  clicks+=c1
  rot_speed=pid_angle.update(RealAngle)
  t2=time.clock()-t1
  s_left=pid_left.update(speed_left)
  s_right=pid_right.update(speed_right)
  driveMotors.set_speed([s_right-rot_speed, s_left+rot_speed])
  print(speed_left,speed_right,RealAngle)
  wiringpi.delayMicroseconds(int((dt-t2)*1e6))

driveMotors.set_speed([0, 0])