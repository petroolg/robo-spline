import numpy as np

#  based on BlueBot and Bosch Toolbox
#  by  O. Certik, V. Smutny, P. Krsek, M. Matousek


def robotBoschikt(robot, pos):
    """ Inverse kinematic task - robot Bosch."""

    pos = np.array(pos).astype(float)
    c = np.sqrt(pos[0] ** 2 + pos[1] ** 2)
    v = (c ** 2 - robot.L1 ** 2 - robot.L2 ** 2) / (2 * robot.L1 * robot.L2)
    if abs(v) > 1.0:
        return []

    b = np.arccos(v) * 180.0 / np.pi
    g = np.arccos((c ** 2 + robot.L1 ** 2 - robot.L2 ** 2) / (2 * robot.L1 * c)) * 180.0 / np.pi

    if pos[0] == 0:
        d = 90.0 * np.sign(pos[1])
    else:
        d = np.arctan(pos[1] / pos[0]) * np.sign(pos[0]) * 180.0 / np.pi

    deg = np.empty((2,4))
    deg[0, 3] = pos[3]
    deg[0, 2] = -pos[2]
    deg[0, 1] = b
    deg[0, 0] = 90.0 - d - g
    deg[1, 3] = pos[3]
    deg[1, 2] = -pos[2]
    deg[1, 1] = -b
    deg[1, 0] = 90.0 - d + g

    if pos[0] != 0:
        deg[:, :2] = deg[:, :2] * np.sign(pos[0])

    if pos[0] < 0:
        p = deg[0, :]
        deg[0, :] = deg[1, :]
        deg[1, :] = p

    return deg