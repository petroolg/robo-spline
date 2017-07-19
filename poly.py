''' Module provides functions for points interpolation using splines of 3rd order '''

import numpy as np
from CRS_commander import Commander
from matplotlib import pyplot as plt


def circle_points(c, #type: Commander
             x=500, y0=250, z0=500, r=50, step=5):

    sol = np.array([[0]*6])
    prev_a = None
    rng = 360/step
    for i in range(rng+1):
        y = y0 + r*np.cos((i*step)/180.0*np.pi)
        z = z0 + r*np.sin((i*step)/180.0*np.pi)
        pos = [x, y, z, 0, 0, 0]
        # print('pos', pos)
        # print('i', i)
        prev_a = move_to_pos(c, pos, prev_a)
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)
    return sol

def move_to_pos(c, pos, prev_pos=None):
    a = c.robot.ikt(c.robot, pos)
    valid_lst = []
    num = -1
    for i in range(len(a)):
        irc = c.anglestoirc(a[i])
        validm = irc > c.robot.bound[0]
        validp = irc < c.robot.bound[1]
        valid = np.logical_and(validm, validp)
        if np.all(valid):
            num = i if num == -1 else num
            valid_lst.append(i)

    if prev_pos is not None:
        min_dist = 10000
        for i, s in enumerate(a):
            dist = np.linalg.norm(np.array(a[i]) - prev_pos)
            if dist < min_dist and i in valid_lst:
                min_dist = dist
                num = i
    # print c.anglestoirc(a[num])
    return c.anglestoirc(a[num])


def cubic_spline(x):
    dim = len(x[0])
    v0 = (4 * x[1] - 3 * x[0] - x[2])/2
    vn = (4 * x[-2] - 3 * x[-1] - x[-3]) / 2

    b = -3 * x[1:-3] + 3* x[3:-1]
    b0 = -3 * x[0] + 3 * x[2] - v0
    bn = -3 * x[-3] + 3 * x[-1] - vn
    b = np.insert(b, 0, b0, axis=0)
    b = np.append(b, [bn], axis=0)
    A = np.eye(x.shape[0] - 2) * 4
    i, j = np.indices(A.shape)
    A[i == j - 1] = 1
    A[i - 1 == j] = 1
    v1ton = np.linalg.inv(A).dot(b)
    v = np.insert(v1ton, 0, v0, axis=0)
    v = np.append(v, [vn], axis=0)

    k0 = [[0]*dim, x[1] - x[0] - v0, v0, x[0]]
    kn = [[0]*dim, -x[-2] +x[-1] + vn, -vn, x[-2]]
    A = 2 * x[1:-2] - 2 * x[2:-1] + v[2:-1] + v[1:-2]
    B = -3 * x[1:-2] + 3 * x[2:-1] - v[2:-1] - 2 * v[1:-2]
    C = v[1:-2]
    D = x[1:-2]

    lst = [A, B, C, D]
    # k = np.vstack([k0, np.hstack([A, B, C, D]), kn])
    for i, m in enumerate(lst):
        lst[i] = np.insert(lst[i], 0, k0[i], axis=0)
        lst[i] = np.append(lst[i], [kn[i]], axis=0)

    [A, B, C, D] = lst
    param_lst = []
    for i in range(len(lst[0])):
        param = np.vstack((C[i], B[i], A[i]))
        param = np.reshape(param.T, [1, 18], order='C')
        param_lst.append(list(param[0]))

    np.save('param', param_lst)
    # print(param_lst)
    return lst

def interpolate(points, graph=False):
    res = 100 #step of x axis

    [A, B, C, D] = cubic_spline(points)

    if graph:
        t = np.arange(0, points.shape[0])
        x = []
        t2 = np.linspace(0, 1, res)
        N = len(A)
        for K in range(N):
            for i in range(len(t2)):
                x.append(np.array(A[K])*t2[i]**3 + np.array(B[K])*t2[i]**2 + np.array(C[K])*t2[i]+ D[K])
        x = np.array(x)[np.newaxis].T
        t3 = np.linspace(0, N, N*res)
        plt.plot(t, points.T[0], t, points.T[1], t, points.T[2], t, points.T[3], t, points.T[4], t, points.T[5],
                 t3, x[0], t3, x[1], t3, x[2], t3, x[3], t3, x[4], t3, x[5] )

        plt.show()
