import numpy as np
from scipy.optimize import minimize
import ADC.IR_distance as DM
import motor_control.motor_class as MC
import time
from accelerometer.BNO055 import BNO055 
   
import shelve
import skfuzzy as fuzz
from skfuzzy import control as ctrl

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

#define membership for angle error
angle_space=linspace(-180,180,720)



angle_error = ctrl.Antecedent(angle_space, 'quality')
angle_error['low'] = fuzz.trimf(angle_error.universe, [-0.5,0,0.5])
angle_error['low'] = fuzz.trimf(angle_error.universe, [-0.5,0,0.5])

inital_angle=bno.read_euler()
time.sleep(0.1)
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


    
