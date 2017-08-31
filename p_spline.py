''' Module provides functions for points interpolation using p-splines '''

# based on Eilers and Marx, Flexible Smoothing with B-splines and Penalties, 1996

import numpy as np
from matplotlib import pyplot as plt

def b_spline(xs, xl, xr, ndx, bdeg):
    dx = (xr - xl)/ndx
    t = xl + dx*np.arange(-bdeg, ndx)
    Bs = np.empty((len(xs), t.shape[0]))
    r = np.arange(1, len(t))
    r = np.append(r, 0)
    T = (0*xs[0] + 1)*t
    for i, x in enumerate(xs):

        X = x*(0*t + 1)
        P = (X - T)/dx
        B = np.array((T <= X) & (X < (T + dx))).astype(float)
        for k in range(1, bdeg+1):
            B = (np.multiply(P, B) + np.multiply((k + 1 - P), B[r]))/k
        Bs[i,:] = B
    return Bs


def p_spline(x, xl, xr, ndx, bdeg, pord, lam, y):
    assert bdeg >= pord, 'Degree of polynomial must be greater than penalty of order'
    B = b_spline(x, xl, xr, ndx, bdeg)
    m, n = B.shape
    D = np.diff(np.eye(n), pord).T
    a = np.linalg.inv(B.T.dot(B) + lam*D.T.dot(D)).dot(B.T.dot(y))
    # yhat = B.dot(a)
    # Q = np.linalg.inv(B.T.dot(B) + lam*D.T.dot(D))
    # s = np.sum((y - yhat)**2)
    # t = np.sum(np.diag(Q.dot(B.T.dot(B))))
    # gcv = s/(m - t) ** 2
    return a


def M_trans(xl, xr, ndx, bdeg):
    assert bdeg == 2 or bdeg == 3, 'Function supports only 2nd and 3rd order polynomials'
    dx = (xr - xl)/ndx;
    T = xl + dx*np.arange(-bdeg,ndx + 2)
    T = T[0:bdeg*2]

    if bdeg == 2:
        a = (T[1] - T[2])*(T[0] - T[2])
        b = (T[1] - T[2])*(T[1] - T[3])

        M = np.array([[1.0, 0.0, 0.0], [-2.0, 1.0, 0.0], [1.0, -1.0, 1.0]])\
            .dot(np.diag([dx**2, dx, 1]))\
            .dot(np.array([[1/a, -2*T[2]/a, T[2]**2/a],
                  [-(1/b + 1/a), (T[1] + T[3])/b + (T[0] + T[2])/a,
                   -T[1]*T[3]/b - T[0]*T[2]/a],
                  [1/b, -2*T[1]/b, T[1]**2/b]]).T)

    if bdeg == 3:
        c = (T[2] - T[3])*(T[1] - T[3])
        d = (T[2] - T[3])*(T[2] - T[4])
        a = (c*(T[0] - T[3]))
        b = (d*(T[2] - T[5]))
        e = (T[2]/d + T[4]/d + T[1]/c + T[3]/c)
        f = (1/d + 1/c)
        g = ((T[2]*T[4])/d + (T[1]*T[3])/c)
        h = (T[1] - T[4])

        M = np.array([[1.0, 0.0, 0.0, 0.0], [-3.0, 1.0, 0.0, 0.0], [3.0, -2.0, 1.0, 0.0], [-1.0, 1.0, -1.0, 1.0]])\
            .dot(np.diag([dx**3, dx**2, dx, 1]))\
            .dot(np.array([[1/a, -3*T[3]/a, 3*T[3]**2/a, -T[3]**3/a],
                  [(-f/h - 1/a), (e/h + (T[4]*f)/h + T[0]/a + (2*T[3])/a),
                   (- g/h - (T[4]*e)/h - T[3]**2/a - (2*T[0]*T[3])/a),
                   (T[4]*g)/h + (T[0]*T[3]**2)/a],
                  [(f/h + 1/b), (- e/h - (T[1]*f)/h - (2*T[2])/b - T[5]/b),
                   (g/h + (T[1]*e)/h + T[2]**2/b + (2*T[2]*T[5])/b),
                   -(T[1]*g)/h - (T[2]**2*T[5])/b],
                  [-1/b, (3*T[2])/b, -(3*T[2]**2)/b, T[2]**3/b]]).T)

    return dx, M


def interpolate(points, num_segments, poly_deg, p_ord, lambda_, graph=True):
    assert poly_deg >= p_ord, 'Degree of polynomial must be greater than penalty of order'
    t = np.arange(0, points.shape[0])

    a = p_spline(t.T, 0.0, float(points.shape[0]-1), num_segments, poly_deg, p_ord, lambda_, points)
    step, Mn = M_trans(0, float(points.shape[0]-1), num_segments, poly_deg)

    param = np.empty((0, points.shape[1]*poly_deg))
    for i in range(num_segments):
        c = np.flipud(Mn.dot(a[:poly_deg + 1])).T
        c = np.reshape(c[:,1:], (points.shape[1]*poly_deg))
        param = np.append(param, [c], axis=0)

    np.save('param_p_spline_%d'%poly_deg, param)


    if graph:
        res = 100  # step of x axis
        if poly_deg == 2:
            t2 = np.array([np.linspace(0, 1, res) ** 2, np.linspace(0, 1, res), np.ones(res)])
        else:
            t2 = np.array([np.linspace(0, 1, res) ** 3, np.linspace(0, 1, res) ** 2, np.linspace(0, 1, res), np.ones(res)])

        x = Mn.dot(a[:poly_deg + 1]).T.dot(t2)
        for K in range(1, num_segments):
            x = np.hstack([x, Mn.dot(a[K:K + poly_deg + 1]).T.dot(t2)])

        t3 = np.linspace(0, num_segments * step, num_segments * res)
        plt.plot(t, points.T[0], t, points.T[1], t, points.T[2], t, points.T[3], t, points.T[4], t, points.T[5])
        plt.plot(t3, x[0], t3, x[1], t3, x[2], t3, x[3], t3, x[4], t3, x[5])

        plt.show()