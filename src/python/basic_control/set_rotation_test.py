import numpy as np
from scipy.optimize import minimize
import ADC.IR_distance as DM
import motor_control.motor_class as MC
import time
from accelerometer.BNO055 import BNO055 
import control.pid as pid      
def opt_dist(set,motor,dm,x):
  motor.set_speed(x)
  return (dm.getDistance()-set)


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
bno.setExternalCrystalUse(True)
  
 # bno.AssistedCalibration()
bno.getCalibrationFromFile(calibrationFile='calibration_data_A.db')
dm=DM.distanceMeas(calibrationFile='calibration_data.db')        
motors=MC.motor_control(0x61,[1,3],[-1,1])
tol=10
a=np.ones(2)
v_o=np.ones(2)
v=v_o
v_o[0]=40
v_o[1]=-40

rotation_desired=90.0
inital_angle=bno.getVector(bno.VECTOR_EULER)



p_left=pid.PID(P=0.5, I=0.0, D=0.1)
p_right=pid.PID(P=0.5, I=0.0, D=0.1)
p.setPoint(0)


#
time.sleep(0.1)
speed=40.0
motors.set_speed(v_o)
angle_diff=100

while abs(angle_diff)>0.1:
  
  angle=bno.getVector(bno.VECTOR_EULER)
  angle_diff =-rotation_desired+compare_angle(inital_angle[0],angle[0])
  delta = p.update(angle_diff)
  v[0]=delta*speed
  v[1]=-delta*speed
  print(abs(angle_diff),delta,v)
  motors.set_speed(v)
  
  distance=dm.getDistance()
  #print(angle_diff )

  time.sleep(0.01)
motors.set_speed(np.zeros(2))


    
