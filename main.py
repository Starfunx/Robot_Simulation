# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt
from Terrain import Terrain as Terrain
from DiffDriveRobot import Robot as Robot
from DiffDriveControl import Robot as RobotControl
from Lidar import Lidar


class PickConsign:
    def __init__(self, robotControl):
        line, = plt.plot([0],[0])
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        self.robotControl = robotControl

    def __call__(self, event):
        print('click', event)
        if(event.xdata != None) and (event.ydata != None):
            self.robotControl.setXC(event.xdata)
            self.robotControl.setYC(event.ydata)
            self.robotControl.setThetaC(2)


terrain_lines =  np.array([[[0,       0], [0,    2000]],
                          [[0,    2000], [3000, 2000]],
                          [[3000, 2000], [3000,    0]],
                          [[3000,    0], [0,       0]],
                          [[1900, 1000], [2200, 1900]]])


terrain = Terrain(terrain_lines)
robot = Robot(1000, 1000, 0)
robotControl = RobotControl(robot.getX(), robot.getY(), robot.getTheta(), robot)
lidar = Lidar([robot.getX(), robot.getY()], 0, 50, np.pi/30, terrain_lines)
pickConsign = PickConsign(robotControl)

dT = 0.001

for t in range(90000):

  #computing
    robot.update(dT)
    robotControl.update(dT)

    if (t*dT)%(1.0/30.0) < 0.001:
        lidar.setX(robot.getX())
        lidar.setY(robot.getY())
        lidar.setTheta(robot.getTheta())
        lidar.fire()

    if (t*dT)%(1.0/30.0) < 0.001:
      # Drawing
        plt.clf()
        plt.axis('equal')
        plt.text(10, 1900, "t: {0:.3f} s".format((t+1)*dT), fontsize=12)
        plt.text(10, 1800, "X: {0:.0f} mm".format(robot.getX()), fontsize=12)
        plt.text(10, 1700, "Y: {0:.0f} mm".format(robot.getY()), fontsize=12)
        plt.text(10, 1600, "T: {0:.3f} rad".format(robot.getTheta()), fontsize=12)

        lidar.draw()
        terrain.draw()
        robot.draw()
        robotControl.draw()
        plt.pause(0.001)

plt.show()
