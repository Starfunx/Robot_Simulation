#coding utf-8
import numpy as np

class DiffDriveRobot(object):
    """docstring for DiffDriveRobot."""

    def __init__(self, wheelRadius, distanceBetweenMotorWheels, robotMass, robotInertia,
                    Kv, R, L, B):
        super(DiffDriveRobot, self).__init__()

        self.Kv = Kv
        self.Kt = Kv
        self.R = R
        self.L = L
        self.B  = B

        self.motors_Voltages = [0.0, 0.0] # [Right, Left]
        self.motors_currents = [0.0, 0.0]
        self.motors_torques = [0.0, 0.0]
        self.motors_velocities = [0.0, 0.0]
        self.motors_positions = [0.0, 0.0]

        self.r = wheelRadius
        self.l = distanceBetweenMotorWheels
        self.m = robotMass
        self.I = robotInertia

        self.angularAcc = 0.
        self.linearAcc = 0.
        self.linearVelocity = 0.
        self.angularVelocity = 0.
        self.theta = 0.
        self.x = 0.
        self.y = 0.

    def update(self,dT):

        E = [self.motors_velocities[0]*self.Kv, self.motors_velocities[1]*self.Kv]

        self.motors_currents[0] += (self.motors_Voltages[0]-E[0] - self.R *self.motors_currents[0])/self.L *dT
        self.motors_currents[1] += (self.motors_Voltages[1]-E[1] - self.R *self.motors_currents[1])/self.L *dT

        self.motors_torques[0] = self.motors_currents[0] * self.Kt - self.motors_velocities[0]*self.B
        self.motors_torques[1] = self.motors_currents[1] * self.Kt - self.motors_velocities[1]*self.B

        F = (self.motors_torques[0] + self.motors_torques[1])/self.r
        M = (self.motors_torques[0] - self.motors_torques[1])*self.l/self.r

        self.linearAcc =  F / self.m
        self.angularAcc =  M / self.I

        self.linearVelocity = self.linearVelocity + self.linearAcc * dT
        self.angularVelocity = self.angularVelocity + self.angularAcc * dT

        dX = self.linearVelocity * dT
        dtheta = self.angularVelocity * dT

        self.theta = self.theta + dtheta
        self.x = self.x + dX * np.cos(self.theta)
        self.y = self.y + dX * np.sin(self.theta)

        self.motors_velocities[0] = (self.linearVelocity + self.angularVelocity*self.l/2)/self.r
        self.motors_velocities[1] = (self.linearVelocity - self.angularVelocity*self.l/2)/self.r

        self.motors_positions[0] += self.motors_velocities[0] * dT
        self.motors_positions[1] += self.motors_velocities[1] * dT

    def getDistBetweenWheels(self):
        return self.l

    def getWheelRadius(self):
        return self.r

    def getMotorsVoltages(self):
        return self.motors_Voltages

    def getMotorCurrents(self):
        return self.motors_currents

    def getWheelsTorque(self):
        return self.motors_torques

    def getWheelsVelocity(self):
        return self.motors_velocities

    def getWheelsAngularPos(self):
        return self.motors_positions

    def getRobotAccelerations(self):
        return [self.linearAcc, self.angularAcc]

    def getRobotVelocities(self):
        return [self.linearVelocity, self.angularVelocity]

    def getRobotPosition(self):
        return [self.x, self.y, self.theta]

    def setMotorVoltages(self, Vr, Vg):
        self.motors_Voltages = [Vr,Vg]
