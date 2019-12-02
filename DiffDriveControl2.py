# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

import Transform as Transform
from DynamicDiffDriveRobotModel import DiffDriveRobot as Robot

class RobotControl(object):
    """docstring for RobotControl."""

    def __init__(self, robot):
        super(RobotControl, self).__init__()

        self.x = 0.
        self.y = 0.
        self.theta = 0.
        self.robot = robot

        self.MEntreAxes = self.robot.getDistBetweenWheels()
        self.MWheelRadius = self.robot.getWheelRadius()
        self.OEntreAxes = self.robot.getDistBetweenWheels()
        self.OWheelRadius = self.robot.getWheelRadius()

        self.lastOdomBuffer = [0.,0.]

        self.xC = self.x
        self.yC = self.y
        self.thetaC = self.theta

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
        dOR = (self.robot.getWheelsAngularPos()[0] - self.lastOdomBuffer[0])* self.OWheelRadius
        dOL = (self.robot.getWheelsAngularPos()[1] - self.lastOdomBuffer[1])* self.OWheelRadius

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

        Kp = 1*10
        Kalpha = 6*10
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


    def update(self, dT):
        self.updateOdometry(dT)
        self.computeError()
        self.setConsign()

    def getPosition(self):
        return [self.x, self.y, self.theta]
    def getCons(self):
        return [self.xC, self.yC]

if __name__== "__main__":
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
    La = 8.e-3
    Ra = 14.2
    c  = 5.e-3

    #frame
    r = 0.025
    l = 0.150
    m = 2
    I = 2.08e-2
    robot = Robot(r, l, m, I, Km, La, Ra, c)
    robotControl = RobotControl(robot)
    robotControl.setX(0.)
    robotControl.setY(0.)
    robotControl.setTheta(0.)
    lidar = Lidar( [robot.getRobotPosition()[0], robot.getRobotPosition()[1]], robot.getRobotPosition()[2], 160, np.pi/100, terrain_lines)

    pickConsign = PickConsign(robotControl)

    Tmax = 15
    for t in range(int(Tmax/dT)):
      #computing
        robot.update(dT)
        robotControl.update(dT)

        if (t*dT*30)%1.0 < 0.002:
            print(robot.getRobotPosition())
            lidar.setX(robot.getRobotPosition()[0]*1e3)
            lidar.setY(robot.getRobotPosition()[1]*1e3)
            lidar.setTheta(robot.getRobotPosition()[2])
            lidar.fire()

          # Drawing
            plt.clf()
            # fig, axs = plt.subplots(2, 2)
            plt.axis('equal')
            plt.text(10, 1900, "t: {0:.3f} s".format((t+1)*dT), fontsize=12)
            # plt.text(10, 1800, "X: {0:.0f} mm".format(robot.getRobotPosition()[0]*1e3), fontsize=12)
            # plt.text(10, 1700, "Y: {0:.0f} mm".format(robot.getRobotPosition()[1]*1e3), fontsize=12)
            # plt.text(10, 1600, "T: {0:.3f} rad".format(robot.getRobotPosition()[2]), fontsize=12)

            lidar.draw()
            ### Terrain Draw
            for linePts in terrain_lines:
                line = np.transpose(linePts)
                plt.plot(line[0], line[1], 'b')
            ### Robot draw
            polygon = np.array([[-150, -150], [-150, 150], [150, 150], [150, -150], [-150, -150]],dtype =float)
            shape2 = np.transpose(Transform.rotate(polygon, robot.getRobotPosition()[2]))
            shape2 = np.transpose(Transform.translate(np.transpose(shape2), robot.getRobotPosition()[0]*1000, robot.getRobotPosition()[1]*1000))
            plt.plot( shape2[0], shape2[1],'g')
            ### Robot control draw
            polygon = np.array([[-150, -150], [-150, 150], [150, 150], [150, -150], [-150, -150]],dtype =float)
            shape2 = np.transpose(Transform.rotate(polygon, robotControl.getPosition()[2]))
            shape2 = np.transpose(Transform.translate(np.transpose(shape2), robotControl.getPosition()[0]*1e3, robotControl.getPosition()[1]*1e3))
            plt.plot( shape2[0], shape2[1], 'r')
            ### Robot control Consign
            plt.plot( robotControl.getCons()[0]*1e3, robotControl.getCons()[1]*1e3 , 'bx')

            plt.pause(0.01)
    plt.show()
