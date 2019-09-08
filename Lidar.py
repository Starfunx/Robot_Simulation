import numpy as np
import matplotlib.pyplot as plt



class Lidar:
    """docstring for Lidar."""

    def __init__(self, position, orientation, nbRays, RaySpacing, map):
        super(Lidar, self).__init__()
        self.nbRays = nbRays
        self.RaySpacing = RaySpacing
        self.map = np.array(map)

        orientations = np.linspace(-nbRays/2*RaySpacing + orientation,
                                   +nbRays/2*RaySpacing + orientation, nbRays)
        orientations = orientations

        self.rays = np.array([np.ones_like(orientations) * position[0],
                              np.ones_like(orientations) * position[1],
                              np.cos(orientations),
                              np.sin(orientations)])
        self.detectedDist = np.array(())
        self.detectedPoints = np.array((1,2), ndmin = 2).transpose()

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

        for i in range(len(self.map)) :
            x1 = self.map[i,0,0]
            y1 = self.map[i,0,1]
            x2 = self.map[i,1,0]
            y2 = self.map[i,1,1]

            denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
            denom = np.ma.masked_equal(denom, 0)

            t = np.ma.masked_outside(((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denom, 0., 1.)
            u = np.ma.masked_less_equal(-((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/denom, 0.)
            if i == 0 :
                t = np.ma.masked_array(t, u.mask)
                u = np.ma.masked_array(u, t.mask)
                minu = u
                intX = x1 + t*(x2-x1)
                intY = y1 + t*(y2-y1)
            else :
                t = np.ma.masked_array(t, u.mask)
                u = np.ma.masked_array(u, t.mask)
                t = np.ma.masked_array(t, (u >= minu).filled(False))

                intXMasked = np.ma.masked_array(intX, (u < minu).filled(False))
                intYMasked = np.ma.masked_array(intY, (u < minu).filled(False))

                intX = np.ma.array(intXMasked.filled(1) * (x1 + t*(x2-x1)).filled(1), mask=(intX.mask * (x1 + t*(x2-x1)).mask))
                intY = np.ma.array(intYMasked.filled(1) * (y1 + t*(y2-y1)).filled(1), mask=(intY.mask * (y1 + t*(y2-y1)).mask))

                minu = np.ma.masked_where(u.filled(9000) < minu, minu)
                minu = np.ma.array(minu.filled(1) * u.filled(1), mask=(minu.mask * u.mask))
        self.detectedDist = np.array(minu)
        self.detectedPoints = np.array(np.array([intX, intY]).transpose())

    def draw(self):
        for i in range(len(self.detectedPoints)):
            line = np.array([[self.rays[0,i], self.rays[1,i]],self.detectedPoints[i]]).transpose()
            plt.plot(line[0], line[1],'C4')


if __name__== "__main__":
    import main
