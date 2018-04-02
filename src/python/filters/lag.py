from math import exp,log


class lag_filter:

    def __init__(self,sample_time,responce_time,sensor_change, intial_value=0):
        '''Simple lag filter which will average previous measurements with a time decay
        Filter can be designed such that a change in sensor value will be become dominant in the signal
        at a given responce time
        '''
        self.filtered=intial_value
        self.k = 1-exp(1- sensor_change)*(sample_time/responce_time)
        self.k_minus = 1-self.k

    def update(self,value):
        self.filtered = self.k*value + self.k_minus*self.filtered

    def value(self):
        return(self.filtered)