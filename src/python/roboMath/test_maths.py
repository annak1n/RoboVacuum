import numpy as np
import movement as mv

from math import pi

w2w=10
wr=0.5

INS =  mv.calc(wr,w2w)

speed=np.zeros(2)

#test case1: wheels rotate equal and oposite
dist= -(w2w*pi/2)
rot = dist/wr
speed[0]=rot
speed[1] =-rot
INS.deltaGlobalX(speed,1)
print(INS.X[0:2])
print(180*INS.X[2]/pi)
