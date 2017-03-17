import numpy as np
from scipy.optimize import minimize
import ADC.IR_distance as DM
import motor_control.motor_class as MC
import time
from accelerometer.BNO055 import BNO055 
import control.pid as pid      
import shelve

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
brush=MC.motor_control(0x60,[2,3,4],[-1,1,1])
brush.set_speed([255,0,0])


db = shelve.open('calibration_data.db')

robot_radius = 15.0
db['robot_radius']=15.0
db.close()

speed_calib_left=np.zeros((255,2))
speed_calib_right=np.zeros((255,2))
buffer_size=10
buff=np.zeros(buffer_size)
for speed in range(255):
  motors.set_speed([0,speed])
  angle1=bno.read_euler()
  t1=time.time() 
  time.sleep(0.015)
  for t in range(buffer_size):
    angle2=bno.read_euler()
    t2=time.time() 
    buff[t]=2*robot_radius*(3.14/180)*(angle2[0]-angle1[0])/(t2-t1)
    t1=t2
    angle1=angle2
    time.sleep(0.015)
  vel=np.median(buff)
  print(vel)
  speed_calib_left[speed]=vel
motors.set_speed([0,0])
time.sleep(0.5)

for speed in range(255):
  motors.set_speed([speed,0]) 
  angle1=bno.read_euler()
  t1=time.time() 
  time.sleep(0.015)
  for t in range(buffer_size):
    angle2=bno.read_euler()
    t2=time.time() 
    buff[t]=2*robot_radius*(3.14/180)*(angle2[0]-angle1[0])/(t2-t1)
    t1=t2
    angle1=angle2
    time.sleep(0.015)
  vel=np.median(buff)
  print(vel)
  speed_calib_right[speed]=vel
motors.set_speed([0,0])

db = shelve.open('calibration_data.db')
db['speed_calib_left']=speed_calib_left
db['speed_calib_right']=speed_calib_right      
db.close()
