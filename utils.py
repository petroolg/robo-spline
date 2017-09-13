import numpy as np


def param_correction(start, params, order):
    pos = np.round(start.copy())
    oldpos = np.round(start.copy())
    pos_not_rounded = np.round(start.copy())
    for i in range(len(params)):
        param = np.reshape(params[i], [order, len(start)], order='F')
        pos += np.sum(np.round(param), axis=0)
        pos_not_rounded += np.sum(param, axis=0)
        diff = pos - pos_not_rounded

        param[0] = np.round(param[0]) - np.round(diff)
        pos = oldpos.copy() + np.sum(param, axis=0)
        oldpos = pos.copy()
        params[i] = np.reshape(np.round(param.T), [1, len(start)*order], order='C')
    return params