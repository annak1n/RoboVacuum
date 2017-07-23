
import numpy as np
from math import sin,cos

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

        self.X = np.zeros(3)
        self.dX = np.zeros(3)
        self.Rho= np.zeros(3)
        self.dRho = np.zeros(2)
        self.J=np.zeros((3,2))
        self.J[0,:]=0.5*wheelRadius
        self.J[1,:]=0.5*wheelRadius
        self.J[2,0]=0.5*(wheelRadius/(0.5*wheel2wheel))
        self.J[2,1]=-0.5*(wheelRadius/(0.5*wheel2wheel))
        self.Jinv=np.ones((2,3))
        self.Jinv[0,2]=(0.5*wheel2wheel)
        self.Jinv[1,2]=-(0.5*wheel2wheel)
        self.Jinv *= 1/wheelRadius

    def deltaGlobalX(self,dRho,dt):
        '''This method can be called to translate the wheel velocities to gloabal velocites
            The equation dX = J*dRho
        '''
        angle=self.X[2]
        tempJ=np.array(self.J,copy=True)
        tempJ[0,:] *= cos(angle)
        tempJ[1,:] *= sin(angle)
        print(tempJ)
        self.dX = tempJ.dot(dRho)
        self.X += self.dX*dt

    def deltaX_to_Wheel(self,dX):
        '''This method can convert global velocities to wheel angular velocities
            dRho = Jinv*dX

        '''
        angle=self.X[2]
        tempJinv=np.array(self.Jinv,copy=True)
        tempJinv[:,0] *= cos(angle)
        tempJinv[:,1] *= sin(angle)
        self.dRho = tempJinv.dot(dX)
        self.Rho += self.dRho * dt