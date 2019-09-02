import numpy as np
import matplotlib.pyplot as plt

class Terrain:
    """docstring for Terrain."""

    def __init__(self, terrain_polygons):
        # super(Terrain, self).__init__()
        self.terrain = terrain_polygons

    def draw(self):
        for polygon in self.terrain:
            lines = np.transpose(polygon)
            plt.plot(lines[0], lines[1])
            pass
