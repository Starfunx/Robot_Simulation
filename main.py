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
                          [[3000,    0], [0,       0]]])

nbpts = 1/0.005*5
trajectory_t = np.linspace(0,20,nbpts*10)
trajectory = np.array([500*np.cos(trajectory_t)+1500,
                        500*np.sin(trajectory_t)+1500])
# trajectory = np.array([trajectory_t*200+700,
#                         np.ones_like(trajectory_t)*1500])

terrain = Terrain(terrain_lines)
robot = Robot(700, 900, 0)
robotControl = RobotControl(robot.getX(), robot.getY(), robot.getTheta(), robot)
lidar = Lidar([robot.getX(), robot.getY()], 0, 50, np.pi/30, terrain_lines)
pickConsign = PickConsign(robotControl)

dT = 0.005
i = 0
robotpos = [[],[]]
Tmax = int(50/dT)
for t in range(Tmax):



  #computing
    robot.update(dT)
    robotControl.update(dT)

    if t*dT > trajectory_t[i]:
        i+=1
        if i>=len(trajectory_t):
            i-=1
        robotControl.setXC(trajectory[0,i])
        robotControl.setYC(trajectory[1,i])

    if (t*dT)%(1.0/30.0) < 0.001:
        lidar.setX(robot.getX())
        lidar.setY(robot.getY())
        lidar.setTheta(robot.getTheta())
        lidar.fire()

    if (t*dT)%(1.0/30.0) < 0.001:
      # Drawing
        plt.clf()
        # plt.axis('equal')
        plt.text(10, 1900, "t: {0:.3f} s".format((t+1)*dT), fontsize=12)
        plt.text(10, 1800, "X: {0:.0f} mm".format(robot.getX()), fontsize=12)
        plt.text(10, 1700, "Y: {0:.0f} mm".format(robot.getY()), fontsize=12)
        plt.text(10, 1600, "T: {0:.3f} rad".format(robot.getTheta()), fontsize=12)
        robotpos[0] = robotpos[0] + [robot.getX()]
        robotpos[1] = robotpos[1] + [robot.getY()]
        # print(robotpos)
        plt.plot(robotpos[0], robotpos[1], 'k+')
        # lidar.draw()
        terrain.draw()
        robot.draw()
        robotControl.draw()
        #draw trajectory
        # plt.plot(trajectory[0], trajectory[1], 'k')


        plt.pause(0.0001)

plt.show()
