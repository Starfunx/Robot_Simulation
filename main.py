import numpy as np
import matplotlib.pyplot as plt
from Terrain import Terrain as Terrain
from DiffDriveRobot import Robot as Robot

terrain_polygons =  [
                        np.array([[0, 0], [0, 2000], [3000, 2000], [3000, 0], [0, 0]])
                    ]

terrain = Terrain(terrain_polygons)
robot = Robot(1000, 1000, 0)
dT = 0.001

robot.setLeftMotorSpeed(1000) #input mm.s-1
robot.setRightMotorSpeed(1000)

for t in range(500):

  #comuting
    robot.update(dT)

  # Drawing
    plt.clf()
    plt.axis('equal')
    plt.text(10, 1900, "t: "+str((t+1)*dT), fontsize=12)
    plt.text(10, 1800, "X: "+str(robot.getX()), fontsize=12)
    plt.text(10, 1700, "Y: "+str(robot.getY()), fontsize=12)
    plt.text(10, 1600, "T: "+str(robot.getTheta()), fontsize=12)

    terrain.draw()
    robot.draw()
    plt.pause(0.001)

plt.show()
