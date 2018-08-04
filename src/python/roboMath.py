from dimensions import ureg
import numpy as np
from math import cos,sin
class DWR_transformations:

    def __init__(self, wheel_to_wheel_distance, wheel_radius):
        '''
        Class for transforming velocities of DWR robot

        '''
        self.wheel_radius = wheel_radius.to('cm').magnitude
        self.wheel_to_wheel_distance=wheel_to_wheel_distance.to('cm').magnitude
        self.radius_between_wheels = 0.5*wheel_to_wheel_distance.to('cm').magnitude

        '''
        Define a matrix which converts global values in x,y,omega into wheel rotations
        This matrix is a mixed unit matrix
        Wheel_values = J_inv * Global_value
        '''
        self.global_to_wheel = np.zeros((2, 3))
        self.create_global_to_wheel(0*ureg.radians)
        self.wheel_to_global = np.zeros((3,2))
        self.create_wheel_to_global(0*ureg.radians)


    def create_global_to_wheel(self,angle):
        '''
        Function which creates the inverse jacobian for the nonholonomic system of equations giverning the DWR


        '''

        c = cos(angle.to('radians').magnitude)
        s = sin(angle.to('radians').magnitude)

        self.global_to_wheel[:,0] = c
        self.global_to_wheel[:,1] = s
        self.global_to_wheel[0,2] = self.radius_between_wheels
        self.global_to_wheel[1,2] = -self.radius_between_wheels
        self.global_to_wheel *= 1.0/self.wheel_radius


    def create_wheel_to_global(self,angle):

        c = cos(angle.to('radians').magnitude)
        s = sin(angle.to('radians').magnitude)
        self.wheel_to_global[0,:]=c
        self.wheel_to_global[1,:]=s
        self.wheel_to_global[2,0]=1.0/self.radius_between_wheels
        self.wheel_to_global[2,1]=-1.0/self.radius_between_wheels
        self.wheel_to_global *= 0.5*self.wheel_radius


    def wheel_speeds(self,angle,global_speed):
        self.create_global_to_wheel(angle)
        return self.global_to_wheel.dot(global_speed)*ureg.cm/ureg.seconds

    def wheel_speeds_split(self,angle,velocity_linear, velocity_angular):
        return(self.wheel_speeds(angle,np.array(
            velocity_linear[0].to('cm/s').magnitude,
            velocity_linear[1].to('cm/s').magnitude,
            velocity_angular.to('rad/s').magnitude
        )))

    def global_speeds(self,angle,wheel_speed):
        self.create_wheel_to_global(angle)
        return self.wheel_to_global.dot(wheel_speed.to('cm/s').magnitude)

    def global_speeds_split(self,angle,wheel_speed):
        X = self.global_speeds(angle,wheel_speed)
        return (X[0:2]*ureg.cm/ureg.seconds , X[2]*ureg.radians/ureg.seconds)

    def rotation_matrix(self,angle):
        c = cos(angle.to('radians').magnitude)
        s = sin(angle.to('radians').magnitude)
        R = np.zeros((2,2))
        R[0,0]=c
        R[1,0]=s
        R[0,1]=-s
        R[1,1]=c
        return R
