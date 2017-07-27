''' Module provides functions for points interpolation using b-splines of 2nd order '''

import numpy as np
from matplotlib import pyplot as plt


def b_spline_2(x):

    n = len(x)
    if n < 5:
        raise Exception('Not enough points for interpolation of 2nd order. Minimal number for spline of 2nd order is 5.')
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

def b_spline_3(x):
    n = len(x)
    if n < 7:
        raise Exception('Not enough points for interpolation. Minimal number for spline of 3rd order is 7.')
    x = np.hstack([np.linspace(0, n - 1, n)[np.newaxis].T, x])
    M0 = np.flip(np.array([[-12, 36, -36, 12],
                   [21, -54, 36, 0],
                   [-11, 18, 0, 0],
                   [2, 0, 0, 0]]),axis=1)

    M1 = np.flip(np.array([[-3, 9, -9, 3],
                   [7, -15, 3, 7],
                   [-6, 6, 6, 2],
                   [2, 0, 0, 0]]),axis=1)

    M2 = np.flip(np.array([[-2, 6, -6, 2],
                   [6, -12, 0, 8],
                   [-6, 6, 6, 2],
                   [2, 0, 0, 0]]),axis=1)

    M3 = np.flip(np.array([[-2, 6, -6, 2],
                   [6, -12, 0, 8],
                   [-7, 6, 6, 2],
                   [3, 0, 0, 0]]),axis=1)

    M4 = np.flip(np.array([[-2, 6, -6, 2],
                   [11, -15, -3, 7],
                   [-21, 9, 9, 3],
                   [12, 0, 0, 0]]),axis=1)


    lst = [1.0/12.0 * x.T[:, 0:4].dot(M0)]
    lst.append(1.0/12.0  * x.T[:, 1:5].dot(M1))

    for i in range(2, n - 5):
        lst.append(1.0/12.0  * x.T[:, i:i + 4].dot(M2))

    lst.append(1.0/12.0  * x.T[:, -5:-1].dot(M3))
    lst.append(1.0/12.0 * x.T[:, -4:].dot(M4))

    param = [[(lst[i][1:])[:, 1:]] for i in range(len(lst))]
    param = np.reshape(param, [len(lst), 18])
    np.save('param_spline_3', param)

    return lst


def interpolate(points, order=2, graph=False):
    res = 100 #step of x axis
    t2 = np.array([])
    if order==2:
        SP = b_spline_2(points) #type: np.ndarray
        t2 = np.array([np.linspace(0, 1, res) ** 0, np.linspace(0, 1, res), np.linspace(0, 1, res) ** 2])
    else:
        SP = b_spline_3(points)  # type: np.ndarray
        t2 = np.array([np.linspace(0, 1, res) ** 0, np.linspace(0, 1, res), np.linspace(0, 1, res) ** 2, np.linspace(0, 1, res) ** 3])

    if graph:
        t = np.arange(0, points.shape[0])

        x = np.array([np.array(SP[0]).dot(t2)][0])
        for K in range(1, len(SP)):
            f = np.array(SP[K]).dot(t2)
            x = np.hstack([x, np.array(SP[K]).dot(t2)])

        #plotting against vector x[0] instead of t
        plt.plot(t, points.T[0], t, points.T[1], t, points.T[2], t, points.T[3], t, points.T[4], t, points.T[5],
                 x[0], x[6], x[0], x[1], x[0], x[2], x[0], x[3], x[0], x[4], x[0], x[5] )

        plt.show()

