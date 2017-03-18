import logging
import sys
import time
import shelve
from Adafruit_BNO055 import BNO055 as AD
import numpy as np
'''
This is a super class for the adafruit BNO055 class. This adds two functions, assisted calibration and get calibration from file.
The shelve module is used to store these n a database file, and to read them out if the appropiate function is called.
'''

class BNO055(AD.BNO055):

  def __init__(self, rst=18, address=0x28, i2c=None, gpio=None,
                 serial_port='/dev/ttyAMA0', serial_timeout_sec=10, **kwargs):
    super(BNO055,self).__init__( rst, address, i2c, gpio,
                 serial_port, serial_timeout_sec, **kwargs)

  def AssistedCalibration(self,calibrationFile='calibration_data.db'):
    '''
      Assisted Calibration, prompts user via the shell to perform validation 
      steps and saves them in a DB. The database name can be give as an argument.
    '''
    self.begin()
    calib_status=[0,0,0,0]
    print('Waiting for Gyroscope to calibrate, do not mov/ne')
    while calib_status[1]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
    print('Gryoscope calibrated /n Calibrating compass, move the sensor in a figure of eight/n')
    while calib_status[3]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
    print('Compass calibrated /n Calibrating accelerometer, move the sensor in 45 degree/n increments in around each axis /n this is really annoying to do.. /n')
    while calib_status[2]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
    print('Accelerometer calibrated /n Calibrating system (whatever that means)/n ')
    while calib_status[0]!=3:
      calib_status=self.get_calibration_status()
      time.sleep(0.01)
      print(calib_status)
    calibration = bno.get_calibration()
    db = shelve.open(calibrationFile)
    db['BNO055_calibration']=calibration
    db.close()    

  def getCalibrationFromFile(self,calibrationFile='calibration_data.db'):
    '''
    Reading the calibration data from a file. Default name can be changed.
    '''
    db = shelve.open(calibrationFile)
    calibration=db['BNO055_calibration']
    db.close()
    self.set_calibration(calibration)
  def setExternalCrystalUse(self,value):
    '''
    This function is for weaning myself off the other python library
    '''
    A=1
    
  def clockStretchBugMode(self,buffer_size=3):
        #function to make datastrcutures in class for dealing with clock stretching bug
        self.old_angles=np.zeros((buffer_size,3))
        self.old_angles_itt=0
        self.old_angles_buff_size=buffer_size
  def readOrientationCS(self):
        #the easiest way I have for dealing with the big is median filtering. A buffer is generated and the angles added. The median angles are returned.
        self.old_angles[self.old_angles_itt,:]=self.read_euler()
        self.old_angles_itt+=1
        if self.old_angles_itt==self.old_angles_buff_size:
              self.old_angles_itt=0
        return( np.median(self.old_angles,axis=0))
  

if __name__ == '__main__':

  '''
  Library when called starts assisted calibration, then just reads off values...
  '''
  bno = BNO055()
  if bno.begin() is not True:
    print "Error initializing device"
    exit()
  time.sleep(1)
  #bno.setExternalCrystalUse(True)
  
  bno.AssistedCalibration()
  bno.getCalibrationFromFile()
  offset=[0,0,0]
  b_size=1000

  #print(temp)
  pos=[0,0,0]
  vel=[0,0,0]
  while True:
    print(bno.read_euler())
    
    
    time.sleep(0.05)
