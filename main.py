import numpy as np
import matplotlib.pyplot as plt
from Terrain import Terrain as Terrain
from DiffDriveRobot import Robot as Robot
from DiffDriveControl import Robot as RobotControl
# from RayCaster import Lidar

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

def main():
    # from RayCaster import Lidar
    terrain_polygons =  [
                            np.array([[0, 0], [0, 2000], [3000, 2000], [3000, 0], [0, 0]])
                        ]

    terrain = Terrain(terrain_polygons)
    robot = Robot(1000, 1000, 0)
    robotControl = RobotControl(1000, 1000, 0, robot)
    # lidar = Lidar([robot.getX(), robot.getY()], 0, 50, np.pi/60, terrain_polygons[0])



    dT = 0.001

    robot.setX(1000)
    robot.setY(1000)
    robot.setTheta(0)

    pickConsign = PickConsign(robotControl)

    for t in range(100000):

      #computing
        robot.update(dT)
        robotControl.update(dT)

        # lidar.fire()
        if (t*dT)%(1/15) < 0.001:
          # Drawing
            plt.clf()
            plt.axis('equal')
            plt.text(10, 1900, "t: {0:.3f} s".format((t+1)*dT), fontsize=12)
            plt.text(10, 1800, "X: {0:.0f} mm".format(robot.getX()), fontsize=12)
            plt.text(10, 1700, "Y: {0:.0f} mm".format(robot.getY()), fontsize=12)
            plt.text(10, 1600, "T: {0:.3f} rad".format(robot.getTheta()), fontsize=12)

            terrain.draw()
            robot.draw()
            robotControl.draw()
            # lidar.draw()
            plt.pause(0.001)

    plt.show()

if __name__ == '__main__':
    main()
