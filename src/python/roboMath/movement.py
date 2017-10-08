
import numpy as np
from math import sin,cos,pi

class calc:
    '''Class to calculate the Global speeds etc. from the wheel rotations
    
    X the global position
    dX the rate of change of this
    Rho the wheel angle in radians s^-1
    dRho the wheel rotation speeds
    J the jacobian for transforming wheel rotations to global speeds
    Jinv the inverse jacobian for this


    '''
    def __init__(self, wheelRadius, wheel2wheel ):

        self.WheelRadius = wheelRadius
        self.ChassisWidth = wheel2wheel
        self.CountsPerRotation=1024
        self.X = np.zeros(3) #positions
        self.dX = np.zeros(3)
        self.Rho= np.zeros(3)
        self.dRho = np.zeros(2)
        self.V = np.zeros(2)
        self.J=np.zeros((3,2))
        self.J[0,:]=0.5*self.WheelRadius
        self.J[1,:]=0.5*self.WheelRadius
        self.J[2,0]=(self.WheelRadius/(self.ChassisWidth ))
        self.J[2,1]=-(self.WheelRadius/(self.ChassisWidth ))
        self.Jinv=np.ones((2,3))
        self.Jinv[0,2]=(0.25*self.ChassisWidth )
        self.Jinv[1,2]=-(0.25*self.ChassisWidth )
        self.Jinv *= 1/self.WheelRadius
        

    def rotWheels(self,counts,gyro,dt):
        self.dRho=((counts/self.CountsPerRotation) * 2 * pi)/dt #radians per second
        self.V = self.dRho * self.WheelRadius
        self.deltaGlobalX(self.dRho,gyro,dt)


    def deltaGlobalX(self,dRho,gyro,dt):
        '''This method can be called to translate the wheel velocities to gloabal velocites
            The equation dX = J*dRho
        '''
        self.X[2]=gyro
        tempJ=np.array(self.J,copy=True)
        tempJ[0,:] *= cos(self.X[2])
        tempJ[1,:] *= sin(self.X[2])
        self.dX = tempJ.dot(dRho)
        self.X += self.dX*dt
        self.X[2]=gyro

    def deltaX_to_Wheel(self,dX, gyro):
        '''This method can convert global velocities to wheel angular velocities
            dRho = Jinv*dX

        '''
        angle=gyro
        tempJinv=np.array(self.Jinv,copy=True)
        tempJinv[:,0] *= cos(angle)
        tempJinv[:,1] *= sin(angle)
        self.dRho = tempJinv.dot(dX)
        self.Rho += self.dRho * dt