import numpy as np
import matplotlib.pyplot as plt

class Rayon:
    """raycaster."""

    def __init__(self, vect, origin):
        super(Rayon, self).__init__()
        self.vect = vect
        self.origin = origin


def getIntersection(ray, pointA, pointB):
    x1 = pointA[0]
    y1 = pointA[1]
    x2 = pointB[0]
    y2 = pointB[1]

    x3 = ray.origin[0]
    y3 = ray.origin[1]
    x4 = ray.origin[0] + ray.vect[0]
    y4 = ray.origin[1] + ray.vect[1]

    denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
    if denom == 0 :
        return np.array([-1])

    t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denom
    u = ((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/denom

    if (t>0 and t<1 and u>0):
        intX =  x1 + t * ( x2 - x1)
        intY =  y1 + t * ( y2 - y1)
        return np.array([intX, intY])
    else:
        return np.array([-1])


class Lidar:
    """docstring for Lidar."""

    def __init__(self, position, orientation, nbRays, spaceBetweenRays, map):
        super(Lidar, self).__init__()
        self.position = position
        self.orientation = orientation
        self.nbRays = nbRays
        self.spaceBetweenRays = spaceBetweenRays
        self.map = map

        self.detectedPoints = np.array((1,2), ndmin = 2).transpose()
        print(self.detectedPoints.shape)

    def setX(self, x):
        self.position[0] = x

    def setY(self, y):
        self.position[1] = y

    def setTheta(self, theta):
        self.orientation = theta

    def fire(self):
        for i in range(self.nbRays):
            rayOrientation = [np.cos(self.orientation - (self.nbRays/2*self.spaceBetweenRays) + i*self.spaceBetweenRays),
                              np.sin(self.orientation - (self.nbRays/2*self.spaceBetweenRays) + i*self.spaceBetweenRays)]
            ray = Rayon(rayOrientation, self.position)
            intersection = getIntersection(ray, self.map[0], self.map[1])
            if np.all(intersection != [-1]) :
                # print(np.transpose(np.array([intersection])))
                self.detectedPoints = np.append(self.detectedPoints, np.transpose([intersection]), axis = 1)
                # print(self.detectedPoints.shape)

    def draw(self):
        # self.detectedPoints = [[0,2],[0,1000]]
        # print('detectedPoints' + str(self.detectedPoints))
        for B in self.detectedPoints.transpose():
            line = np.array([self.position, B]).transpose()
            # print(line)
            plt.plot(line[0], line[1],'C4')

        pass
