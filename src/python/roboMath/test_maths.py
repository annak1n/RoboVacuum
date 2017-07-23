import unittest
import numpy as np
import movement as mv

from math import pi


class TestRoboMathMethods(unittest.TestCase):
    
    def setUp(self):
        self.w2w=10
        self.wr=0.5
        self.INS =  mv.calc(self.wr,self.w2w)
        self.steps=400
        self.dt=1/self.steps

    def test_moveForward(self):
        speed=np.zeros(2)
        dist= 20 #
        anglular_movement = (dist/self.wr) # angle radians
        dOmega = anglular_movement/(self.dt*self.steps) #radians per second
        speed[0] = dOmega
        speed[1] = dOmega
        for t in range(0,self.steps):
            self.INS.deltaGlobalX(speed,self.dt)

        self.assertTrue(abs(self.INS.X[2])<0.000001)
        self.assertTrue(abs(self.INS.X[1])<0.000001)
        self.assertTrue(abs(self.INS.X[0]-dist)<0.000001)

    def test_rotation(self):
        speed=np.zeros(2)

        #test case1: wheels rotate equal and oposite
        r=(self.w2w/2)
        dist= r * pi/2 #arc length
        anglular_movement = (dist/self.wr) # angle radians
        dOmega = anglular_movement/(self.dt*self.steps) #radians per second
        speed[0] = dOmega
        speed[1] = -dOmega

        for t in range(0,self.steps):
            self.INS.deltaGlobalX(speed,self.dt)
        self.assertTrue(abs(self.INS.X[2]-pi/2)<0.000001)
        self.assertTrue(abs(self.INS.X[1])<0.000001)
        self.assertTrue(abs(self.INS.X[0])<0.000001)

    def test_moveUpward(self):
        speed=np.zeros(2)
        dist= 20 #
        anglular_movement = (dist/self.wr) # angle radians
        dOmega = anglular_movement/(self.dt*self.steps) #radians per second
        speed[0] = dOmega
        speed[1] = dOmega
        self.INS.X[2]=pi/2
        for t in range(0,self.steps):
            self.INS.deltaGlobalX(speed,self.dt)
            
        self.assertTrue(abs(self.INS.X[2]-pi/2)<0.000001)
        self.assertTrue(abs(self.INS.X[1]-dist)<0.000001)
        self.assertTrue(abs(self.INS.X[0])<0.000001)

    def test_loop_CW(self):
        speed=np.zeros(2)

        #test case1: wheels rotate equal and oposite
        r=(self.w2w)
        dist= r * 2 *pi #arc length
        anglular_movement = (dist/self.wr) # angle radians
        dOmega = anglular_movement/(self.dt*self.steps) #radians per second
        speed[0] = dOmega
        speed[1] = 0

        for t in range(0,self.steps):
            self.INS.deltaGlobalX(speed,self.dt)
            print(self.INS.X)
        self.assertTrue(abs(self.INS.X[2]-2*pi)<0.000001)
        self.assertTrue(abs(self.INS.X[1])<0.000001)
        self.assertTrue(abs(self.INS.X[0])<0.000001)

if __name__ == '__main__':
    unittest.main()