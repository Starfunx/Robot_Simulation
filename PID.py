#coding utf-8
class PID(object):
    """docstring for PID."""

    def __init__(self, Kp, Ti, Td, minOut, maxOut):
        super(PID, self).__init__()
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
        self.integral = 0.
        self.last_error = 0.

        self.minOut = minOut
        self.maxOut = maxOut

    def update(self, error):
        proportional = error*self.Kp
        derivate = (error - self.last_error)*self.Td
        self.integral += error*self.Ti
        self.last_error = error
        if self.integral > self.maxOut:
            self.integral = self.maxOut
        elif self.integral < self.minOut:
            self.integral = self.minOut
        output = proportional + derivate + self.integral
        if output > self.maxOut:
            output = self.maxOut
        elif output < self.minOut:
            output = self.minOut
        return output

    def setPID(self, Kp, Ti, Td):
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
