from rotary_encoder import wheel_encoder as encoder
import motor_control.motor_class as MC
import time
from control.pid import PID
from math import pi, copysign

def calc_speed(clicks,dt):
  return(((float(clicks)/1024.0)*2.0*pi*3.0)/dt)


driveMotors = MC.motor_group([0x61, 0x61], [1, 3], [1, -1])


###this is just playing around with PID and motor controler
enc=encoder.WheelEncoder()
print(enc.read_counters())
#driveMotors.set_speed([127, 0])
t1=time.time()
t2=t1
total_clicks=2**11
speed=20.0
clicks=0;
P=0.5
I=0.25
D=0.2
pid_left=PID(P=P,I=I,D=D)
pid_right=PID(P=P,I=I,D=D) 
pid_left.setPoint(speed)
pid_right.setPoint(speed)
dt=0.02
s_left=+0
s_right=+0
while clicks<total_clicks: 
  t1=time.time()
  c1,c2=enc.read_counters()
  speed_left=copysign(calc_speed(c2,dt),s_left)
  speed_right=copysign(calc_speed(c1,dt),s_right)
  clicks+=c1
  t2=time.time()-t1
  
  s_left=pid_left.update(speed_left)
  s_right=pid_right.update(speed_right)
  driveMotors.set_speed([pid_right.update(speed_right), s_left])
  print(speed_left,speed_right)
  time.sleep(dt-t2)
  
driveMotors.set_speed([0, 0])