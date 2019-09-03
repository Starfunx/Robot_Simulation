import numpy as np
import matplotlib.pyplot as plt

import Transform as Transform

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

    def __init__(self, x, y, theta):
        super(Robot, self).__init__()
        self.polygon = np.array([[-150, -150], [-150, 150], [150, 150], [150, -150], [-150, -150]],dtype =float)

        self.x = x
        self.y = y
        self.theta = theta

        self.V = 0
        self.Omega = 0

        self.MweelG = Wheel()
        self.MweelD = Wheel()
        self.MEntreAxes = 200

        self.OweelG = Wheel()
        self.OweelD = Wheel()
        self.OEntreAxes = 250

  # mutateurs
    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setTheta(self, theta):
        self.theta = theta

    def setLeftMotorSpeed(self, speed):
        self.MweelG.setSpeed(speed)

    def setRightMotorSpeed(self, speed):
        self.MweelD.setSpeed(speed)

  #asscenseurs
    def getLeftEncoderDist(self, dT):
        return self.OweelG.getDist(dT)

    def getRightEncoderDist(self, dT):
        return self.OweelD.getDist(dT)

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getTheta(self):
        return self.theta

  #autres methodes
    #fonctions traduisant le fonctionment du robot (modèle)
    def updateRobotData(self, dT):
        #hypothèse de non glissement des roues odométriques
        EO = self.OEntreAxes
        EM = self.MEntreAxes
        A = np.matrix([ [    1, -EO/2],
                        [    1, +EO/2]], dtype = float) # [VOg, VOd]t <= A*[V, Omega]t
        B = np.matrix([ [  1/2,   1/2],
                        [-1/EM, +1/EM]], dtype = float) # [V, Omega]t <= A*[VMg, VMd]t
        VMg = self.MweelG.getSpeed()
        VMd = self.MweelD.getSpeed()
        VM = np.matrix([VMg, VMd], dtype = float)
        R = B*np.transpose(VM)
        self.V = np.array(R)[0][0]
        self.Omega = np.array(R)[1][0]
        R = A*R
        self.OweelG.setSpeed(np.array(R)[0][0])
        self.OweelD.setSpeed(np.array(R)[1][0])
        self.theta = self.theta + self.Omega *dT
        if(self.theta <= np.pi): self.theta = self.theta + 2*np.pi
        if(self.theta > np.pi): self.theta = self.theta - 2*np.pi
        self.x = self.x + self.V*dT*np.cos(self.theta)
        self.y = self.y + self.V*dT*np.sin(self.theta)

    def draw(self):
        shape2 = np.transpose(Transform.rotate(self.polygon, self.theta))
        shape2 = np.transpose(Transform.translate(np.transpose(shape2), self.x, self.y))
        plt.plot( shape2[0], shape2[1])


    def update(self, dT):
        self.updateRobotData(dT)



if __name__== "__main__":
    import main.py
