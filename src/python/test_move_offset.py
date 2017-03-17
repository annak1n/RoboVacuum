import numpy as np
from scipy.optimize import minimize
import ADC.IR_distance as DM
import motor_control.motor_class as MC
import time

        
def opt_dist(set,motor,dm,x):
  motor.set_speed(x)
  return (dm.getDistance()-set)
     
dm=DM.distanceMeas(calibrationFile='calibration_data.db')        
motors=MC.motor_control(0x61,[1,3],[-1,1])
tol=10
a=np.ones(2)
v=np.ones(2)
while True:
  print(v)
  tol=0.1*opt_dist(30,motors,dm,v)
  print(tol)
  v=a*tol
  time.sleep(0.01)


    
