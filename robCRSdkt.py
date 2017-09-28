import numpy as np

def robCRSdkt(robot, pos):
    T = np.eye(4)
    pos = np.array(pos) / 180.0 * np.pi

    for i in range(6):
        tz = np.eye(4)
        tz[2,3] = robot.d[i]
        rz = np.eye(4)
        o = -robot.offset[i]
        rz[:2, :2] = np.array([[np.cos(o+pos[i]), -np.sin(o+pos[i])],
                               [np.sin(o+pos[i]), np.cos(o+pos[i])]])
        tx = np.eye(4)
        tx[0, 3] = robot.a[i]
        rx = np.eye(4)
        a = robot.alpha[i]
        rx[1:3, 1:3] = np.array([[np.cos(a), -np.sin(a)],
                               [np.sin(a), np.cos(a)]])
        T = T.dot(tz).dot(rz).dot(tx).dot(rx)
    T = T.dot(robot.tool)
    coord = T[:3,3]
    a3 = np.arctan2(T[1, 0], T[0, 0])/np.pi * 180
    a4 = np.arcsin(-T[2,0])/np.pi * 180
    a5 = np.arctan2(T[2,1], T[2,2])/np.pi * 180
    coord = np.hstack((coord, a3,a4, a5))
    return coord
