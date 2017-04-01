import numpy as np
import time
class encoder():

    def __init__(self):
        #here we need to define the conection details, 

        self.counts=np.zeros(2,dtype=int) #counts are 2 32 bit integers
        self.lastReadTime=time.time()
        self.dt
    def update(self):
        
        Read_time =time.time()
        self.counts=1#here it should comminicate with counter
        
        self.dt = Read_time - self.lastReadTime

        return(self.counts/self.dt)

