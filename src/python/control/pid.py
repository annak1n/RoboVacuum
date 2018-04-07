# The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller
#
# cnr437@gmail.com
#
#######	Example	#########
#
# p=PID(3.0,0.4,1.2)
# p.setPoint(5.0)
# while True:
#     pid = p.update(measurement_value)
#
#

from math import pi
def GetAngleDifference(A, B):
    '''

    '''
    A.ito('radians')
    B.ito('radians')
    difference = min(A - B,2*pi -A + B,key=abs)
    '''while difference < -pi:
        difference += 2.0*pi
    while difference > pi:
        difference -= 2.0*pi
    '''
    return difference


class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500, Angle=False, unit = 1):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator * unit
        self.Integrator = Integrator * unit
        self.Integrator_max = Integrator_max * unit
        self.Integrator_min = Integrator_min * unit
        self.Angle = Angle
        self.set_point = 0.0 * unit
        self.error = 0.0 * unit

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        if self.Angle == True:
            self.error = GetAngleDifference(self.set_point, current_value)
        else:
            self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        #self.Integrator = 0
        self.Derivator = 0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator
