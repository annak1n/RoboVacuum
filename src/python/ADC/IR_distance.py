from ABE_ADCPi import ADCPi
from ABE_helpers import ABEHelpers
import time
import os
import numpy as np
from scipy.interpolate import interp1d
import shelve
import time


class distanceMeas():
  
  
  def __init__(self,add1=0x6A,add2=0x6B,channel=1,calibrationFile=False):
    i2c_helper = ABEHelpers()
    self.bus = i2c_helper.get_smbus()
    self.adc = ADCPi(self.bus, add1, add2, 12)
    self.calibrationFile=calibrationFile
    self.channel=channel
    
    if calibrationFile!=False:
      db = shelve.open(calibrationFile)
      calibration=db['distanceIR_calibration']
      db.close() 
      self.interpolator=interp1d(calibration[:-1,0], calibration[:-1,1], kind='cubic',bounds_error=False)
    else:
      self.interpolator=self.noInterpolation
    
    
  def noInterpolation(self,Voltage):
    return(Voltage)
    
  def getDistance(self):
    voltage=self.adc.read_voltage(self.channel)
    return(self.interpolator(voltage))

  def calibrate(self,calibrationFile='calibration_data.db'):
    self.calibration = np.empty( shape=(0, 0) )
    diff=1
    entries=0
    old_voltage=0
    while diff>0:
      distance = float(raw_input("Please enter distance to sensor (cm):/n always decrease distance "))
      voltage=float(self.adc.read_voltage(self.channel))
      diff=voltage-old_voltage
      if diff>0:
        self.calibration=np.append(self.calibration,[voltage, distance])
        entries+=1
        old_voltage=voltage
        print (self.calibration.reshape([entries,2]))
      print (self.calibration.reshape([entries,2]))
    self.calibration=self.calibration.reshape([entries,2])
    db = shelve.open(calibrationFile)
    db['distanceIR_calibration']=self.calibration
    db.close()    
    self.interpolator==interp1d(self.calibration[:,0], self.calibration[:,1], kind='cubic')
    
    
if __name__ == '__main__':
  dm=distanceMeas(calibrationFile='calibration_data.db')
  for i in range(10):
    print(dm.getDistance())
    time.sleep(0.5)
    
  #dm.calibrate()

  for i in range(30):
    print(dm.getDistance())
    time.sleep(0.5)
  
      