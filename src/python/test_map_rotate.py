import numpy as np
from scipy.optimize import minimize
import ADC.IR_distance as DM
import motor_control.motor_class as MC
import time
from accelerometer.BNO055 import BNO055 
        
def opt_dist(set,motor,dm,x):
  motor.set_speed(x)
  return (dm.getDistance()-set)

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
v=np.ones(2)
v[0]=+60
v[1]=-60

motors.set_speed(v)
for i in range(500):
  
  angle=bno.getVector(bno.VECTOR_EULER)
  distance=dm.getDistance()
  
  print(angle[0],",",float(distance))
 
  time.sleep(0.02)
motors.set_speed(np.zeros(2))


    
