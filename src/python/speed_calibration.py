import numpy as np
from scipy.optimize import minimize
import ADC.IR_distance as DM
import motor_control.motor_class as MC
import time
from accelerometer.BNO055 import BNO055 
import control.pid as pid    
import shelve
from scipy.interpolate import interp1d

import atexit



def compare_angle(angle1,angle2):
  difference=angle1-angle2
  while difference < -180:
    difference+=360
  while difference > 180:
    difference -= 360
  return difference
bno = BNO055()
if bno.begin() is not True:
  print "Error initializing device"
  exit()
time.sleep(1)

  
 # bno.AssistedCalibration()
bno.getCalibrationFromFile(calibrationFile='calibration_data_A.db')
dm=DM.distanceMeas(calibrationFile='calibration_data.db')        
motors=MC.motor_control(0x61,[1,3],[-1,1])
def stop_motor():
  motors.set_speed([0,0])

atexit.register(stop_motor)
tol=10
a=np.ones(2)
v_o=np.ones(2)
v=np.ones(2)
speed=100
v_o[0]=speed
v_o[1]=speed

inital_angle=bno.read_euler()

db = shelve.open('calibration_data.db')
speed_calib_left=db['speed_calib_left']
speed_calib_right=db['speed_calib_right']     
db.close()
x=np.arange(0,255)
print len(np.asarray(speed_calib_left[:,0]).flatten())
f_left = interp1d(np.asarray(speed_calib_left[:,0]).flatten(),x,bounds_error=False)
f_right = interp1d(-1.0*np.asarray(speed_calib_right[:,0]).flatten(),x,bounds_error=False)

real_speed=(speed_calib_left[speed,0]+speed_calib_right[speed,0])*0.5
#v_o[0]=f_left(real_speed)
#v_o[1]=f_right(real_speed)
print v_o
p_left=pid.PID(P=0.02, I=0.0, D=0.0)
p_right=pid.PID(P=2, I=0.0, D=0.0)
#p_left.setPoint(real_speed)
p_right.setPoint(real_speed)
radius=15.0

#
v_o[0]=speed
v_o[1]=speed

motors.set_speed(v_o)
time.sleep(0.5)
distance=100
t1=time.time() 
while distance>10:
  
  angle=bno.read_euler()
  t2=time.time()
  angle_diff =compare_angle(inital_angle[0],angle[0])
  if angle_diff>10:
    angle_diff=10
  v[0]=(1-(angle_diff/10))*(speed)
  #print (v[0], (1-(angle_diff/100)),v_o[0])
  v[1]=(1+(angle_diff/10))*(speed)
  

  print(v,v_o,angle_diff)


  #print(angle_diff,delta,v)
  motors.set_speed(v)
  old_distance=distance
  distance=dm.getDistance()
  #print(angle_diff )
  if np.isnan(distance):
    distance=100

 
  time.sleep(0.01)
motors.set_speed(np.zeros(2))


    
