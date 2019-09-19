# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt

class Lidar(object):
    """docstring for Lidar."""

    def __init__(self, position, orientation, nbRays, RaySpacing, map):
        super(Lidar, self).__init__()
        self.nbRays = nbRays
        self.RaySpacing = RaySpacing
        self.map = np.array(map)

        self.orientations = np.linspace(-nbRays/2*RaySpacing + orientation,
                                   +nbRays/2*RaySpacing + orientation, nbRays)
        self.rays = np.array([np.ones_like(self.orientations) * position[0],
                              np.ones_like(self.orientations) * position[1],
                              np.cos(self.orientations),
                              np.sin(self.orientations)])
        self.detectedDist = np.array(())

    def setX(self, x):
        self.rays[0] = np.ones_like(self.rays[0]) * x

    def setY(self, y):
        self.rays[1] = np.ones_like(self.rays[1]) * y

    def setTheta(self, theta):
        orientations = np.linspace(-self.nbRays/2*self.RaySpacing + theta,
                                   +self.nbRays/2*self.RaySpacing + theta, self.nbRays)
        self.rays[2] = np.cos(orientations)
        self.rays[3] = np.sin(orientations)

    def fire(self):
        intersections = self.getIntersections()

    def getIntersections(self):
        x3 = self.rays[0]
        y3 = self.rays[1]
        x4 = self.rays[0] + self.rays[2]
        y4 = self.rays[1] + self.rays[3]

        minu = np.ones_like(self.rays[0])*5000
        for i in range(len(self.map)) :
            x1 = self.map[i,0,0]
            y1 = self.map[i,0,1]
            x2 = self.map[i,1,0]
            y2 = self.map[i,1,1]

            denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
            denom = np.ma.masked_equal(denom, 0)

            t = np.ma.masked_outside(((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denom, 0., 1.)
            u = np.ma.masked_less_equal(-((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/denom, 0.)
            u = np.ma.masked_array(u, t.mask)

            minu = np.ma.masked_where(u.filled(5000) < minu, minu)
            u = np.ma.masked_array(u , np.logical_not(minu.mask))
            minu = np.ma.array(minu.filled(1) * u.filled(1), mask=(minu.mask * u.mask))
        self.detectedDist = np.array(minu)
        return [self.orientations, self.detectedDist]

    def draw(self):
        line = np.array([[self.rays[0], self.rays[0]+self.detectedDist*self.rays[2]],[self.rays[1], self.rays[1]+self.detectedDist*self.rays[3]]])
        plt.plot(line[0], line[1],'C4')

if __name__== "__main__":
    import main
