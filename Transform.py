import numpy as np

def rotate (points, theta):
    points = np.array(points)
    NewCol = np.array(np.ones((np.shape(points)[0], 1)))
    points = np.append(points, NewCol, axis=1)

    Rmatrix = np.matrix([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])

    points = np.array(np.dot(Rmatrix, points.transpose())[:-1,:].transpose())
    return points

def translate (points, x, y):
    points = np.array(points)
    NewCol = np.array(np.ones((np.shape(points)[0], 1)))
    points = np.append(points, NewCol, axis=1)

    Tmatrix = np.matrix([[1, 0, x],
                         [0, 1, y],
                         [0, 0, 1]],dtype = float)
    points = np.array(np.dot(Tmatrix, points.transpose())[:-1,:].transpose())
    return points
