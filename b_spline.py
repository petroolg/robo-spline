''' Module provides functions for points interpolation using b-splines of 2nd order '''

import numpy as np
from matplotlib import pyplot as plt


def b_spline_2(x):

    n = len(x)
    x = np.hstack([np.linspace(0, n-1, n)[np.newaxis].T, x])
    M0 = np.array([[2, -4, 2], [0, 4, -3], [0, 0, 1]])
    M1 = np.array([[1, -2, 1], [1, 2, -2], [0, 0, 1]])
    M2 = np.array([[1, -2, 1], [1, 2, -3], [0, 0, 2]])

    lst = [0.5*x.T[:,0:3].dot(M0)]

    for i in range(1,n-3):
        lst.append(0.5*x.T[:,i:i+3].dot(M1))

    lst.append(0.5 * x.T[:,-3:].dot(M2))

    param = [[(lst[i][1:])[:, 1:]] for i in range(len(lst))]
    param = np.reshape(param, [len(lst), 12])
    np.save('param_spline_2', param)

    return lst

def interpolate(points, graph=False):
    res = 100 #step of x axis

    SP = b_spline_2(points) #type: np.ndarray

    if graph:
        t = np.arange(0, points.shape[0])

        t2 = np.array([np.linspace(0, 1, res)**0, np.linspace(0, 1, res), np.linspace(0, 1, res)**2])
        x = np.array([np.array(SP[0]).dot(t2)][0])
        for K in range(1, len(SP)):
            f = np.array(SP[K]).dot(t2)
            x = np.hstack([x, np.array(SP[K]).dot(t2)])

        #plotting against vector x[0] instead of t
        plt.plot(t, points.T[0], t, points.T[1], t, points.T[2], t, points.T[3], t, points.T[4], t, points.T[5],
                 x[0], x[6], x[0], x[1], x[0], x[2], x[0], x[3], x[0], x[4], x[0], x[5] )

        plt.show()

