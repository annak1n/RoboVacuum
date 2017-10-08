import numpy as np
from skimage.draw import line
class map():

    def __init__(self,size=(5000,5000), pixel_width=5, sensor_limit=120):
        size =np.round(np.array(size)/pixel_width)
        self.data = np.zeros(size,dtype=bool)
        self.pixel_width=pixel_width

    
    def addPoint(self,X,):
        X=np.round(X/self.pixel_width)
        self.data[X[0],X[1]]=True

    def checkLine(self,X,Y):
        X=np.round(X/self.pixel_width)
        Y=np.round(Y/self.pixel_width)

        rr, cc = line(X[0], X[1], Y[0], Y[1])
        vals = self.data[rr,cc]
        intersect=np.argwhere(vals!=0)

        if len(intersect):
            if len(len(intersect))>0:
                intersect=intersect[0]
            return(np.array(rr[intersect],cc[intersect]))
        else:
            return(False)
