from math import exp,sqrt,atan, tan,cos,sin, pi
from my_pint import ureg
import numpy as np
from time import time


class observation:
    def __init__(self,distance,angle, position = np.array([0,0])*ureg.cm):
        self.distance =distance
        self.angle = -pi/2 + angle.to('radians')
        self.position = position
    def OPP(self,x,theta):
        cone = np.exp((-1/sqrt(2))*((theta)/(10*ureg.degrees))**2)
        if self.distance>119*ureg.cm:
            incidence=0.1*np.ones(x.shape)
        else:
            incidence = 0.8*np.exp((-1/sqrt(2))*((self.distance-x).to('cm').magnitude/(2.5))**2) - 0.4
        opp=0.5 + cone*incidence
        m1 = (x>self.distance)*(opp<0.5)
        opp[m1]=0.5
        
        return opp
    
    def global_OPP(self, coarse=5*ureg.cm):
        rect = int(round(((self.distance + 2*2.5*ureg.cm)/coarse + 1).magnitude))
        width = int((tan((10*ureg.degrees).to('radians'))*rect))+1
        X, Y = np.meshgrid(range(1,rect), range(-width,width+1))*coarse
        theta = np.arctan(Y/X)*ureg.radians
        mip = self.OPP(X,theta)
        return mip
        
    def global_OPP_ptcl(self,coarse=5*ureg.cm):
        mip = self.global_OPP(coarse=coarse)
        MASK = (mip<0.45) +  (mip>0.55)
        vals = mip[MASK]
        coords = np.argwhere(MASK)
        c = cos(self.angle)
        s = sin(self.angle)
        R = np.array([[c,-s],[s,c]])
        coords =np.dot(coords,R.T)
        coords[:,0] += self.position[0]/coarse
        coords[:,1] += self.position[1]/coarse
        return (np.round(coords).astype(int),vals)
        

class mapper:
    
    def __init__(self,size, pixel_width=5*ureg.cm):
        self.map = np.ones(size)*0.5
        self.coarse = pixel_width
    def update(self,observation):
        t1 = time()
        pv = observation.global_OPP_ptcl(coarse=self.coarse)
        X = pv[0][:,0]
        Y = pv[0][:,1]
        old=self.map[X,Y]
        self.map[X,Y] = (pv[1]*old)/(pv[1]*old +(1-pv[1])*(1-old))
        print(time()-t1)
