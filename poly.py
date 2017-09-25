''' Module provides functions for points interpolation using splines of 3rd order '''

import numpy as np
from utils import param_correction

# Tri Diagonal Matrix Algorithm solver
def TDMAsolver(d):

    n = len(d)
    b = 4*np.ones_like(d)
    for i in range(1, n):
        mc = 1/b[i-1]
        b[i] -= mc
        d[i] -= mc*d[i-1]

    x = np.empty_like(d)
    x[-1] = d[-1]/b[-1]

    for il in range(n-2, -1, -1):
        x[il] = (d[il]-x[il+1])/b[il]

    return x


def cubic_spline(x):
    n, dim = x.shape

    v0 = (4 * x[1] - 3 * x[0] - x[2]) / 2
    vn = (4 * x[-2] - 3 * x[-3] - x[-1]) / 2  # ?

    b = -3 * x[1:-3] + 3 * x[3:-1]
    b0 = -3 * x[0] + 3 * x[2] - v0
    bn = -3 * x[-3] + 3 * x[-1] - vn
    b = np.vstack((b0, b, bn))

    v1ton = TDMAsolver(b)
    v = np.vstack((v0, v1ton, vn))

    k0 = [[0]*dim, x[1] - x[0] - v0, v0, x[0]]
    kn = [[0]*dim, x[-1] - x[-2] - v[-2], v[-2], x[-2]]  # ?
    A = 2 * x[1:-2] - 2 * x[2:-1] + v[2:-1] + v[1:-2]
    B = - 3 * x[1:-2] + 3 * x[2:-1] - v[2:-1] - 2 * v[1:-2]
    C = v[1:-2]
    D = x[1:-2]

    lst = [A, B, C, D]

    for i, m in enumerate(lst):
        lst[i] = np.vstack((k0[i], lst[i], kn[i]))

    return lst


def interpolate(points):
    [A, B, C, _] = cubic_spline(points)

    param_lst = []

    for i in range(points.shape[0] - 1):
        param = np.vstack((C[i], B[i], A[i]))
        param = np.reshape(param.T, [points.shape[1]*3], order='C')
        param_lst.append(param)

    param_lst = param_correction(points[0], param_lst, 3)
    np.save('params/param_poly', param_lst)
