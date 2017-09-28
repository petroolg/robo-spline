import numpy as np

def showBoschRose(c,):
    rad = 70.0
    k = 7.0 / 4.0
    ts = np.linspace(0, 8 * np.pi, 300)
    z = 10
    sol = np.zeros((1, 4))
    points = []
    for t in ts:
        x = rad * np.cos(t * k) * np.cos(t)
        y = rad * np.cos(t * k) * np.sin(t) + 300.0
        points.append([x, y, z, 0])
    prev_a = None
    for p in points:
        # print( cy + np.cos(a)*r, cz + np.sin(a)*r)
        prev_a = c.move_to_pos(p, prev_a, relative=False, move=True)
        if prev_a is None:
            return None, None
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)

