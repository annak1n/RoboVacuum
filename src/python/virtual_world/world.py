import numpy as np

'''
Virtual world for testing software and algorithms indepdent of hardware

This file defines a simple "world" where there are a list of points and edges

'''

class world:

    def __init__(self):
        self.points=[]
        self.edges=[]

    def simple_square(self,bl,tr):
        start = len(self.points)
        self.points.append(bl)
        self.points.append((tr[0],bl[1]))
        self.points.append(tr)
        self.points.append(bl[0],tr[0])
        self.edges.append([start-1,start])
        self.edges.append([start,start+1])
        self.edges.append([start+1,start+2])
        self.edges.append([start+2,start+3])