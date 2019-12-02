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

    def update(self):
        proportional = error*Kp
        derivate = error*Td
        integral = (error - last_error)*Ti
		if (integral > maxOut)
            integral = maxOut;
        else if (integral < minOut)
            integral = minOut;
        output = proportional + derivate + integral
		if (output > maxOut)
            output = maxOut;
        else if (output < minOut)
            output = minOut;
		return output;

    def setPID(self, Kp, Ti, Td):
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
