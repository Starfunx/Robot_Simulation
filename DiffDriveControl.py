import numpy as np
import matplotlib.pyplot as plt

import Transform as Transform
import DiffDriveRobot

class Wheel:
    """docstring for Wheel."""

    def __init__(self):
        super(Wheel, self).__init__()
        self.speed = 0

    def setSpeed(self, speed):
        self.speed = speed

    def getSpeed(self):
        return self.speed

    def getDist(self, dT):
        return self.speed * dT

class Robot(object):
    """docstring for Robot."""

    def __init__(self, x, y, theta, robot):
        super(Robot, self).__init__()
        self.polygon = np.array([[-150, -150], [-150, 150], [150, 150], [150, -150], [-150, -150]],dtype =float)

        self.x = x
        self.y = y
        self.theta = theta
        self.robot = robot
        self.MEntreAxes = 200
        self.OEntreAxes = 250

        self.xC = x
        self.yC = y
        self.thetaC = theta

        self.XErr = 0
        self.YErr = 0
        self.ThetaErr = 0

        self.DistErr = 0
        self.CapErr = 0

  # mutateurs
    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setTheta(self, theta):
        self.theta = theta

    def setXC(self, xC):
        self.xC = xC

    def setYC(self, yC):
        self.yC = yC

    def setThetaC(self, thetaC):
        self.thetaC = thetaC

  # asscenseurs
    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getTheta(self):
        return self.theta

  #autres methodes
    #fonctions traduisant le fonctionment du robot (modèle)
    def updateOdometry(self, dT):
        dOG = self.robot.getLeftEncoderDist(dT)
        dOR = self.robot.getRightEncoderDist(dT)

        dTheta = (dOR - dOG)/self.OEntreAxes
        dXrobot = (dOR + dOG)/2

        self.theta = self.theta + dTheta
        self.x = self.x + dXrobot*np.cos(self.theta)
        self.y = self.y + dXrobot*np.sin(self.theta)

    def computeError(self):
        self.XErr = self.xC - self.x
        self.YErr = self.yC - self.y
        self.ThetaErr = self.thetaC - self.theta

        self.DistErr = np.sqrt(np.square(self.XErr)+np.square(self.YErr))
        self.CapErr = np.arctan2(self.YErr, self.XErr)-self.theta

        if(self.CapErr <= np.pi): self.CapErr = self.CapErr + 2*np.pi
        if(self.CapErr > np.pi): self.CapErr = self.CapErr - 2*np.pi

    def setConsign(self):
        VMG = self.DistErr*1 - self.CapErr*800
        VMD = self.DistErr*1 + self.CapErr*800
        self.robot.setLeftMotorSpeed(VMG)
        self.robot.setRightMotorSpeed(VMD)

    def draw(self):
        shape2 = np.transpose(Transform.rotate(self.polygon, self.theta))
        shape2 = np.transpose(Transform.translate(np.transpose(shape2), self.x, self.y))
        plt.plot( shape2[0], shape2[1])
        plt.plot( self.xC, self.yC , 'bx')


    def update(self, dT):
        self.updateOdometry(dT)
        self.computeError()
        self.setConsign()

if __name__== "__main__":
    import main.py
