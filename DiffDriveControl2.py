# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

import Transform as Transform
from Robot import Robot as Robot
from Motor import Motor as Motor

class RobotControl(object):
    """docstring for RobotControl."""

    def __init__(self, x, y, theta, robot):
        super(RobotControl, self).__init__()
        self.polygon = np.array([[-150, -150], [-150, 150], [150, 150], [150, -150], [-150, -150]],dtype =float)

        self.x = x
        self.y = y
        self.theta = theta
        self.robot = robot
        self.MEntreAxes = 1.
        self.OEntreAxes = 1.

        self.lastOdomBuffer = [0.,0.]

        self.xC = x
        self.yC = y
        self.thetaC = theta

        self.XErr = 0.
        self.YErr = 0.
        self.ThetaErr = 0.

        self.DistErr = 0.
        self.CapErr = 0.

        self.alpha = []
        self.thetaa = []
        self.DistErra = []
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
        dOR = self.robot.getWheelsAngularPos()[0] - self.lastOdomBuffer[0]
        dOL = self.robot.getWheelsAngularPos()[1] - self.lastOdomBuffer[1]

        self.lastOdomBuffer[0] = self.robot.getWheelsAngularPos()[0]
        self.lastOdomBuffer[1] = self.robot.getWheelsAngularPos()[1]

        dXrobot = (dOR + dOL)/2
        dTheta = (dOR - dOL)/self.OEntreAxes

        self.theta = self.theta + dTheta
        if(self.theta <= -np.pi): self.theta = self.theta + 2*np.pi
        if(self.theta > np.pi): self.theta = self.theta - 2*np.pi

        self.x = self.x + dXrobot*np.cos(self.theta)
        self.y = self.y + dXrobot*np.sin(self.theta)

    def computeError(self): # Equations 11 & 12
        self.XErr = self.xC - self.x
        self.YErr = self.yC - self.y
        self.ThetaErr = self.thetaC - self.theta #unused

        Kp = 1
        Kalpha = 6
        alpha = np.arctan2(self.YErr, self.XErr)-self.theta
        if alpha <= -np.pi: alpha+= 2*np.pi
        if alpha > +np.pi: alpha-= 2*np.pi

        self.thetaa.append(self.theta)
        self.alpha.append(alpha)
        self.DistErr = Kp*np.sqrt(self.XErr**2 + self.YErr**2)*np.cos(alpha)
        # self.CapErr = Kp*np.sin(alpha)*np.cos(alpha) + Kalpha*alpha
        self.CapErr = Kalpha*np.sin(alpha)*np.cos(alpha)

    def setConsign(self):
        V = self.DistErr
        Omega = self.CapErr
        VMD = (V + Omega * self.MEntreAxes/2)/1#1 = wheelRadius
        VMG = (V - Omega * self.MEntreAxes/2)/1
        self.robot.setMotorVoltages(VMD, VMG)

    def draw(self):
        shape2 = np.transpose(Transform.rotate(self.polygon, self.theta))
        shape2 = np.transpose(Transform.translate(np.transpose(shape2), self.x*1e3, self.y*1e3))
        plt.plot( shape2[0], shape2[1])
        plt.plot( self.xC*1e3, self.yC*1e3 , 'bx')


    def update(self, dT):
        self.updateOdometry(dT)
        self.computeError()
        self.setConsign()



def main():
    # coding: utf-8
    import numpy as np
    import matplotlib.pyplot as plt
    from Terrain import Terrain as Terrain
    from Lidar import Lidar



    class PickConsign:
        def __init__(self, robotControl):
            line, = plt.plot([0],[0])
            self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
            self.robotControl = robotControl

        def __call__(self, event):
            print('click', event)
            if(event.xdata != None) and (event.ydata != None):
                self.robotControl.setXC(event.xdata*1e-3)
                self.robotControl.setYC(event.ydata*1e-3)
                self.robotControl.setThetaC(2)


    terrain_lines =  np.array([[[0,       0], [0,    2000]],
                              [[0,    2000], [3000, 2000]],
                              [[3000, 2000], [3000,    0]],
                              [[3000,    0], [0,       0]]])


    terrain = Terrain(terrain_lines)

    dT = 0.001
## Robot definition
    # motors
    Km = 12./(260 * 2*np.pi/60.)
    Kb = 12./(260 * 2*np.pi/60.)

    La = 8.e-3
    Ra = 14.2

    J  = 1.5e-3
    c  = 5.e-0

    motorR = Motor(Km, La, Ra, J, c, Kb)
    motorL = Motor(Km, La, Ra, J, c, Kb)

    #frame
    r = 1.
    l = 1.
    m = 0.002
    I = 0.0025
    robot = Robot(r, l, m, I, motorR, motorL)

    robotControl = RobotControl(robot.getPosition()[0], robot.getPosition()[1], robot.getPosition()[2], robot)
    lidar = Lidar([robot.getPosition()[0], robot.getPosition()[1]], 0, 50, np.pi/30, terrain_lines)
    pickConsign = PickConsign(robotControl)
    robotpos = [[],[]]
    Tmax = int(15/dT)

    time = []
    thetaR = []
    thetaL = []
    for t in range(Tmax):

        time.append(t)
        thetaL.append(robot.getWheelsAngularPos()[0])
        thetaR.append(robot.getWheelsAngularPos()[1])

      #computing
        robot.update(dT)
        robotControl.update(dT)

        # if (t*dT)%(1.0/30.0) < 0.001:
        #     lidar.setX(robot.getPosition()[0])
        #     lidar.setY(robot.getPosition()[1])
        #     lidar.setTheta(robot.getPosition()[2])
        #     lidar.fire()

        if (t*dT)%(1.0/30.0) < 0.001:
          # Drawing
            plt.clf()
            # plt.axis('equal')
            plt.text(10, 1900, "t: {0:.3f} s".format((t+1)*dT), fontsize=12)
            plt.text(10, 1800, "X: {0:.0f} mm".format(robot.getPosition()[0]), fontsize=12)
            plt.text(10, 1700, "Y: {0:.0f} mm".format(robot.getPosition()[1]), fontsize=12)
            plt.text(10, 1600, "T: {0:.3f} rad".format(robot.getPosition()[2]), fontsize=12)
            robotpos[0] = robotpos[0] + [robot.getPosition()[0]]
            robotpos[1] = robotpos[1] + [robot.getPosition()[1]]
            # print(robotpos)
            plt.plot(robotpos[0], robotpos[1], 'k+')
            # lidar.draw()
            terrain.draw()
            robot.draw()
            robotControl.draw()
            #draw trajectory
            # plt.plot(trajectory[0], trajectory[1], 'k')

            plt.pause(0.01)



    fig, ax2 = plt.subplots()
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('theta wheels (rad)')
    ax2.plot(time, thetaR, 'r')
    ax2.plot(time, thetaL, 'b')
    plt.show()



if __name__== "__main__":
    main()
