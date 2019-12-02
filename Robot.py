# coding: utf-8

import numpy as np
from Motor import Motor as Motor
import Transform
import matplotlib.pyplot as plt

class Robot(object):
    """docstring for Robot.
    r : wheel radius in mm
    l : distance betweed two motors wheels in mm
    m : robot mass in Kg
    I : robot z inertia in Kg.m^2
    """

    def __init__(self, r, l, m, I, motorR, motorL):
        super(Robot, self).__init__()
        self.motorR = motorR
        self.motorL = motorL

        self.polygon = np.array([[-150, -150], [-150, 150], [150, 150], [150, -150], [-150, -150]],dtype =float)

        self.r = r
        self.l = l
        self.m = m
        self.I = I

        self.torqueR = 0.
        self.torqueL = 0.

        self.omegaR = 0.
        self.omegaL = 0.

        self.thetaR = 0.
        self.thetaL = 0.


        self.angularAcc = 0.
        self.linearAcc = 0.

        self.linearVelocity = 0.
        self.angularVelocity = 0.

        self.theta =0.
        self.x =0.
        self.y =0.

    def update(self,dT):
        self.motorR.setAngularVelocity(self.omegaR)
        self.motorL.setAngularVelocity(self.omegaL)

        self.motorR.update(dT)
        self.motorL.update(dT)

        self.torqueR = self.motorR.getTorque()
        self.torqueL = self.motorL.getTorque()

        F = (self.torqueR + self.torqueL)/self.r
        M = (self.torqueR - self.torqueL)*self.l/self.r

        self.linearAcc =  F / self.m
        self.angularAcc =  M / self.I

        self.linearVelocity = self.linearVelocity + self.linearAcc * dT
        self.angularVelocity = self.angularVelocity + self.angularAcc * dT

        dX = self.linearVelocity * dT
        dtheta = self.angularVelocity * dT

        self.theta = self.theta + dtheta
        self.x = self.x + dX * np.cos(self.theta)
        self.y = self.y + dX * np.sin(self.theta)

        self.omegaR = (self.linearVelocity + self.angularVelocity*self.l/2)/self.r
        self.omegaL = (self.linearVelocity - self.angularVelocity*self.l/2)/self.r

        self.thetaR = self.thetaR + self.omegaR * dT
        self.thetaL = self.thetaL + self.omegaL * dT

    def getDistBetweenWheels():
        return self.l
    def getWheelRadius():
        return self.r

    def setMotorTorques(self, torqueR, torqueL):
        self.torqueR = torqueR
        self.torqueL = torqueL

    def getlinearAcceleartion(self):
        return self.angularAcc

    def getAngularAcceleartion(self):
        return self.linearAcc

    def getlinearVelocity(self):
        return self.linearVelocity

    def getAngularVelocity(self):
        return self.angularVelocity

    def getPosition(self):
        return [self.x, self.y, self.theta]

    def setPosition(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def getWheelsAngularVelocities(self):
        return [self.omegaR, self.omegaL]

    def getWheelsAngularPos(self):
        return [self.thetaR, self.thetaL]

    def setMotorVoltages(self, VR, VL):
        self.motorR.setVoltage(VR)
        self.motorL.setVoltage(VL)

    def draw(self):
        shape2 = np.transpose(Transform.rotate(self.polygon, self.theta))
        shape2 = np.transpose(Transform.translate(np.transpose(shape2), self.x*1000, self.y*1000))
        plt.plot( shape2[0], shape2[1])


if __name__== "__main__":
    import matplotlib.pyplot as plt
    dT = 0.001
    time = np.arange(0,10,dT)

    VoltageR = np.heaviside(time-0.5,1)*12
    VoltageL = np.heaviside(time-0.5,1)*12

    # motors
    Km = 12./(260 * 2*np.pi/60.)
    Kb = 12./(260 * 2*np.pi/60.)

    La = 8.e-3
    Ra = 14.2

    J  = 1.5e-3
    c  = 5.e-3

    motorR = Motor(Km, La, Ra, J, c, Kb)
    motorL = Motor(Km, La, Ra, J, c, Kb)

    #frame
    r = 100.
    l = 100.
    m = 100.
    I = 100.
    robot = Robot(r, l, m, I, motorR, motorL)

    X = []
    Y = []
    theta = []

    for i in range(len(time)):
        t = time[i]

        robot.setMotorVoltages(VoltageR[i], VoltageL[i])
        robot.update(dT)

        X.append(robot.getPosition()[0])
        Y.append(robot.getPosition()[1])
        theta.append(robot.getPosition()[2])

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.plot(X, Y, 'k+')

    fig, ax2 = plt.subplots()
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('theta (rad)')
    ax2.plot(time, theta, 'y')
    ax2.plot(time, X, 'r')
    ax2.plot(time, Y, 'g')

    plt.show()
