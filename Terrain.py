import numpy as np
import matplotlib.pyplot as plt

class Terrain:
    """docstring for Terrain."""

    def __init__(self, terrain_polygons):
        # super(Terrain, self).__init__()
        self.terrain = terrain_polygons

    def draw(self):
        for linePts in self.terrain:
            line = np.transpose(linePts)
            plt.plot(line[0], line[1], 'b')


if __name__== "__main__":
    import main
