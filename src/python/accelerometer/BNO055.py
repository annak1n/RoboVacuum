import logging
import sys
import time
import shelve
from Adafruit_BNO055 import BNO055 as AD

'''
This is a super class for the adafruit BNO055 class. This adds two functions, assisted calibration and get calibration from file.
'''

class BNO055(AD.BNO055):

  def __init__(self, rst=18, address=0x28, i2c=None, gpio=None,
                 serial_port='/dev/ttyAMA0', serial_timeout_sec=10, **kwargs):
    super(BNO055,self).__init__( rst, address, i2c, gpio,
                 serial_port, serial_timeout_sec, **kwargs)

  def AssistedCalibration(self,calibrationFile='calibration_data.db'):
    self.begin()
    calib_status=[0,0,0,0]
    print('Waiting for Gyroscope to calibrate, do not move')
    while calib_status[1]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
    print('Gryoscope calibrated /n Calibrating compass, move the sensor in a figure of eight')
    while calib_status[3]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
    print('Compass calibrated /n Calibrating accelerometer, move the sensor in 45 degree increments in around each axis')
    while calib_status[2]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
    print('Accelerometer calibrated /n Calibrating system, ')
    while calib_status[0]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
      print(calib_status)
    calibration = bno.get_calibration()
    db = shelve.open(calibrationFile)
    db['BNO055_calibration']=calibration
    db.close()    

  def getCalibrationFromFile(self,calibrationFile='calibration_data.db'):
    db = shelve.open(calibrationFile)
    calibration=db['BNO055_calibration']
    db.close()
    self.set_calibration(calibration)
  def setExternalCrystalUse(self,value):
    A=1
    
    
  

if __name__ == '__main__':
  bno = BNO055()
  if bno.begin() is not True:
    print "Error initializing device"
    exit()
  time.sleep(1)
  #bno.setExternalCrystalUse(True)
  
 # bno.AssistedCalibration()
  bno.getCalibrationFromFile()
  offset=[0,0,0]
  b_size=1000

  #print(temp)
  pos=[0,0,0]
  vel=[0,0,0]
  while True:
    print(bno.read_euler())
    
    
    time.sleep(0.05)
