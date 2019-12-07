#coding utf-8
import numpy as np

class DiffDriveRobot(object):
    """docstring for DiffDriveRobot."""

    def __init__(self, wheelRadius, MotorWheelsDist, #geometric properties
                       robotMass=None, robotInertia=None, #mass properties
                       Kv=None, R=None, L=None, B=None, # motor porperties
                       odometricWheelsRadius=None, odometricWheelsDist=None, encoderRes=None #odometric properties
                       ):

        self._Kv = Kv
        self._Kt = Kv
        self._R = R
        self._L = L
        self._B  = B

        self._motVoltages = np.array([0., 0.]) # [Right, Left]
        self._motCurrents = np.array([0., 0.])
        self._motTorques = np.array([0., 0.])
        self._motAngVels = np.array([0., 0.])
        self._motPos = np.array([0., 0.])

        self._motorsWheelRadius = wheelRadius
        self._motorsWheelsDist = MotorWheelsDist
        self._robotMass = robotMass
        self._robotInertia = robotInertia

        self._robotAcc = np.array([0., 0.]) #in terrain coords
        self._robotVelocity = np.array([0., 0.]) #in terrain coords
        self._robotPos = np.array([0., 0., 0.]) #x,y,theta in terrain coords

        self._odomsWheelRadius = wheelRadius
        self._odomsWheelsDist = MotorWheelsDist
        self._encoderResolution = encoderRes
        self._encoderPos = np.array([0., 0.])
        self._encoderLastGivenPos = np.array([0., 0.])

    def update(self,dT):

        E = self._motAngVels*self._Kv#[self.motors_velocities[0]*self.Kv, self.motors_velocities[1]*self.Kv]

        self._motCurrents += (self._motVoltages-E - self._R*self._motCurrents)/self._L *dT
        # self.motors_currents[0] += (self.motors_Voltages[0]-E[0] - self.R *self.motors_currents[0])/self.L *dT
        # self.motors_currents[1] += (self.motors_Voltages[1]-E[1] - self.R *self.motors_currents[1])/self.L *dT

        self._motTorques = self._motCurrents * self._Kt - self._motAngVels*self._B
        # self.motors_torques[0] = self.motors_currents[0] * self.Kt - self.motors_velocities[0]*self.B
        # self.motors_torques[1] = self.motors_currents[1] * self.Kt - self.motors_velocities[1]*self.B

        # [F,M]=
        F = (self._motTorques[0] + self._motTorques[1])/self._motorsWheelRadius
        M = (self._motTorques[0] - self._motTorques[1])*self._motorsWheelsDist/self._motorsWheelRadius

        self._robotAcc = np.array([F / self._robotMass, M / self._robotInertia])
        # self.linearAcc =  F / self.m
        # self.angularAcc =  M / self.I

        self._robotVelocity += self._robotAcc * dT
        # self.linearVelocity = self.linearVelocity + self.linearAcc * dT
        # self.angularVelocity = self.angularVelocity + self.angularAcc * dT

        [dX, dtheta] = self._robotVelocity * dT
        # dX = self.linearVelocity * dT
        # dtheta = self.angularVelocity * dT

        self._robotPos[0] += dX * np.cos(self._robotPos[2] + dtheta/2)
        self._robotPos[1] += dX * np.sin(self._robotPos[2] + dtheta/2)
        self._robotPos[2] += dtheta
        # self.x = self.x + dX * np.cos(self.theta + dtheta/2)
        # self.y = self.y + dX * np.sin(self.theta + dtheta/2)
        # self.theta = self.theta + dtheta


        self.motAngVels[0] = (self._robotVelocity[0] + self._robotVelocity[1]*self._motorsWheelsDist/2)/self._motorsWheelRadius
        self.motAngVels[1] = (self._robotVelocity[0] - self._robotVelocity[1]*self._motorsWheelsDist/2)/self._motorsWheelRadius
        # self.motors_velocities[0] = (self.linearVelocity + self.angularVelocity*self._motorsWheelsDist/2)/self._motorsWheelRadius
        # self.motors_velocities[1] = (self.linearVelocity - self.angularVelocity*self._motorsWheelsDist/2)/self._motorsWheelRadius

        self._motPos += self._motAngVels * dT
        # self.motors_positions[0] += self.motors_velocities[0] * dT
        # self.motors_positions[1] += self.motors_velocities[1] * dT

    def updateOdometry(arg):
        pass

    def motVoltages():
        doc = "The motVoltages property."
        def fget(self):
            return self._motVoltages
        def fset(self, value):
            self._motVoltages = np.array(value)
        def fdel(self):
            del self._motVoltages
        return locals()
    motVoltages = property(**motVoltages())

    def motCurrents():
        doc = "The motCurrents property."
        def fget(self):
            return self._motCurrents
        def fset(self, value):
            self._motCurrents = value
        def fdel(self):
            del self._motCurrents
        return locals()
    motCurrents = property(**motCurrents())

    def motTorques():
        doc = "The motTorques property."
        def fget(self):
            return self._motTorques
        def fset(self, value):
            self._motTorques = value
        def fdel(self):
            del self._motTorques
        return locals()
    motTorques = property(**motTorques())

    def motAngVels():
        doc = "The motAngVels property."
        def fget(self):
            return self._motAngVels
        def fset(self, value):
            self._motAngVels = value
        def fdel(self):
            del self._motAngVels
        return locals()
    motAngVels = property(**motAngVels())

    def motPos():
        doc = "The motPos property."
        def fget(self):
            return self._motPos
        def fset(self, value):
            self._motPos = value
        def fdel(self):
            del self._motPos
        return locals()
    motPos = property(**motPos())

    def motorsWheelRadius():
        doc = "The motorsWheelRadius property."
        def fget(self):
            return self._motorsWheelRadius
        def fset(self, value):
            self._motorsWheelRadius = value
        def fdel(self):
            del self._motorsWheelRadius
        return locals()
    motorsWheelRadius = property(**motorsWheelRadius())

    def motorsWheelsDist():
        doc = "The motorsWheelsDist property."
        def fget(self):
            return self._motorsWheelsDist
        def fset(self, value):
            self._motorsWheelsDist = value
        def fdel(self):
            del self._motorsWheelsDist
        return locals()
    motorsWheelsDist = property(**motorsWheelsDist())

    def robotMass():
        doc = "The robotMass property."
        def fget(self):
            return self._robotMass
        def fset(self, value):
            self._robotMass = value
        def fdel(self):
            del self._robotMass
        return locals()
    robotMass = property(**robotMass())

    def robotInertia():
        doc = "The robotInertia property."
        def fget(self):
            return self._robotInertia
        def fset(self, value):
            self._robotInertia = value
        def fdel(self):
            del self._robotInertia
        return locals()
    robotInertia = property(**robotInertia())

    def robotAcc():
        doc = "The robotAcc property."
        def fget(self):
            return self._robotAcc
        def fset(self, value):
            self._robotAcc = value
        def fdel(self):
            del self._robotAcc
        return locals()
    robotAcc = property(**robotAcc())

    def robotVelocity():
        doc = "The robotVelocity property."
        def fget(self):
            return self._robotVelocity
        def fset(self, value):
            self._robotVelocity = value
        def fdel(self):
            del self._robotVelocity
        return locals()
    robotVelocity = property(**robotVelocity())

    def robotPos():
        doc = "The robotPos property."
        def fget(self):
            return self._robotPos
        def fset(self, value):
            self._robotPos = value
        def fdel(self):
            del self._robotPos
        return locals()
    robotPos = property(**robotPos())

    def odomsWheelRadius():
        doc = "TodomsWheelRadiushe  property."
        def fget(self):
            return self._odomsWheelRadius
        def fset(self, value):
            self._odomsWheelRadius = value
        def fdel(self):
            del self._odomsWheelRadius
        return locals()
    odomsWheelRadius= property(**odomsWheelRadius())

    def odomsWheelsDist():
        doc = "odomsWheelsDistThe  property."
        def fget(self):
            return self._odomsWheelsDist
        def fset(self, value):
            self._odomsWheelsDist = value
        def fdel(self):
            del self._odomsWheelsDist
        return locals()
    odomsWheelsDist= property(**odomsWheelsDist())

    def encoderResolution():
        doc = "ThencoderResolutione  property."
        def fget(self):
            return self._encoderResolution
        def fset(self, value):
            self._encoderResolution = value
        def fdel(self):
            del self._encoderResolution
        return locals()
    encoderResolution= property(**encoderResolution())

    def encoderPos():
        doc = "The doencoderPos property."
        def fget(self):
            return selencoderPosf._
        def fset(self, value):
            self._encoderPos = value
        def fdel(self):
            del self._encoderPos
        return locals()
    encoderPos= property(**encoderPos())

    def encoderLastGivenPos():
        doc = "The encoderLastGivenPos property."
        def fget(self):
            return self._encoderLastGivenPos
        def fset(self, value):
            self._encoderLastGivenPos = value
        def fdel(self):
            del self._encoderLastGivenPos
        return locals()
    encoderLastGivenPos= property(**encoderLastGivenPos())
