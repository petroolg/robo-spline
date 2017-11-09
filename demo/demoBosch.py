import time
import numpy as np
from matplotlib import pyplot as plt
import poly
from graph import Graph
import os

from CRS_commander import Commander
from robotCRS import robCRS97
from robotBosch import robotBosch

from PIL import Image, BmpImagePlugin
from svgpathtools import svg2paths, wsvg, Path

# np.seterr(all='raise')

def rose_st(c, x0, y0, z0, f_x, f_y, cond, repeats, tl,N):
    # N = tl*2*30
    # print N
    ts = np.linspace(-tl, tl, N)
    points, lengths = [], []
    last_x, last_y = f_x(0), f_y(0)
    angles = np.linspace(0, 2*(repeats-1)/repeats*np.pi, repeats)
    for a in angles:
        T = np.array([[np.cos(a), -np.sin(a)],[np.sin(a), np.cos(a)]])

        for t in ts:
            if cond(t):
                x = f_x(t)
                y = f_y(t)
                x, y = T.dot([[x], [y]])
                points.append([x, y, z0, 0])
                lengths.append(np.sqrt((last_x-x)*(last_x-x) + (last_y-y)*(last_y-y))[0])
                last_x, last_y = x, y

    points = np.array(points)
    points[:, 0] = points[:, 0] + x0
    points[:, 1] = points[:, 1] + y0

    sol = points2irc(c, points)
    # print lengths

    return sol, points, lengths

def points2irc(c, points):
    prev_a = None
    sol = np.empty((1, 4))
    for p in points:
        # print( cy + np.cos(a)*r, cz + np.sin(a)*r)
        prev_a = c.move_to_pos(p, prev_a, relative=False, move=False)
        if prev_a is None:
            return None, None
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)
    return sol


def showBoschRose(c):
    if not os.path.isdir('params'):
        os.mkdir('params')

    params_list = []
    order = 3
    start_points = []
    length = []

    # cond = lambda t: np.cos(1.88 * t) > 0
    # f_x = lambda t: 40.0 * np.power(np.cos(1.88 * t), 2.5)
    # f_y = lambda t: 20.0 * np.sin(np.sin(4.3 * t)) * np.cos(1.88 * t)
    # sol, points, lengths = rose_st(c, 0, 300, 300, f_x, f_y, cond, 7.0, 4.2, 250)
    #
    # length.append(lengths)
    # poly.interpolate(sol)
    # params_list.append(np.load('params/param_poly.npy'))
    # plt.plot(points[:, 0], points[:, 1])
    # start_points.append(np.copy(points[0]))

    # e = Graph(sol)
    # e.show_gui()

    H = 300

    cond = lambda t: True
    f_x = lambda t: 30.0*np.cos(7.0/4.0*t)*np.cos(t)
    f_y = lambda t: 30.0*np.cos(7.0/4.0*t)*np.sin(t)
    sol, points, lengths = rose_st(c, 0, 300, 325, f_x, f_y, cond, 1.0, 4*np.pi, 300)

    length.append(lengths)
    params = poly.interpolate(sol)
    params_list.append(params)
    plt.plot(points[:, 0], points[:, 1])
    start_points.append(np.copy(points[0]))

    cond = lambda t: True
    f_x = lambda t: 30.0 * np.cos(2.0 / 5.0 * t) * np.cos(t)
    f_y = lambda t: 30.0 * np.cos(2.0 / 5.0 * t) * np.sin(t)
    sol, points, lengths = rose_st(c, 75, 380, 325, f_x, f_y, cond, 1.0, 5 * np.pi, 400)

    # e = Graph(sol)
    # e.show_gui()

    length.append(lengths)
    params = poly.interpolate(sol)
    # params_list.append(np.load('params/param_poly.npy'))
    plt.plot(points[:, 0], points[:, 1])
    start_points.append(np.copy(points[0]))

    cond = lambda t: True
    f_x = lambda t: 30.0 * np.cos(2.0 / 9.0 * t) * np.cos(t)
    f_y = lambda t: 30.0 * np.cos(2.0 / 9.0 * t) * np.sin(t)
    sol, points, lengths = rose_st(c, 0, 380, 325, f_x, f_y, cond, 1.0, 9 * np.pi, 400)

    length.append(lengths)
    params = poly.interpolate(sol)
    params_list.append(params)
    plt.plot(points[:, 0], points[:, 1])
    start_points.append(np.copy(points[0]))

    cond = lambda t: True
    f_x = lambda t: 30.0 * np.cos(6.0 / 2.0 * t) * np.cos(t)
    f_y = lambda t: 30.0 * np.cos(6.0 / 2.0 * t) * np.sin(t)
    sol, points, lengths = rose_st(c, 75, 300, 325, f_x, f_y, cond, 1.0, 0.5* np.pi, 150)

    length.append(lengths)
    params = poly.interpolate(sol)
    params_list.append(params)
    plt.plot(points[:, 0], points[:, 1])
    start_points.append(np.copy(points[0]))

    cond = lambda t: True
    f_x = lambda t: 40.0/2 * np.sin(6.04*t/3)/(0.3+np.sin(t)**2)
    f_y = lambda t: 70.0/2 * np.power(np.sin(6.04*t), 4) * np.cos( t)
    sol, points, lengths = rose_st(c, 0, 300, 300, f_x, f_y, cond, 7.0, 3.12, 350)

    length.append(lengths)
    poly.interpolate(sol)
    params_list.append(np.load('params/param_poly.npy'))
    plt.plot(points[:, 0], points[:, 1])
    start_points.append(np.copy(points[0]))

    e = Graph(sol)
    e.show_gui()

    # cond = lambda t: True
    # f_x = lambda t: 30.0 * np.power(np.sin(8.78*t), 6)
    # f_y = lambda t: 40.0 * np.sinh(np.sinh(np.sin(2.94*t)))*np.power(np.cos(8.78*t), 2)
    # sol, points, lengths = rose_st(c, -200, 350, 300, f_x, f_y, cond, 7.0, .54, 300)
    #
    # length.append(lengths)
    # poly.interpolate(sol)
    # params_list.append(np.load('params/param_p_spline_3.npy'))
    # plt.plot(points[:, 0], points[:, 1])
    # start_points.append(np.copy(points[0]))

    plt.show()

    for start, params, le in zip(start_points, params_list, length):
        c.move_to_pos([start[0], start[1], 300, start[3]])
        prev_a = c.move_to_pos(start)
        c.wait_ready(sync=True)
        for i in range(len(params)):
            c.splinemv(params[i], order=order, min_time=le[i+1]*10)
            print(int(le[i+1]*10))
        c.wait_ready(sync=True)
        c.move_to_pos([start[0], start[1], 300, start[3]])
        print('completed')
        start = time.time()
        c.wait_ready()
        print(time.time() - start)


# part to file and bottom left corner
def draw_svg(path, blc):
    paths, attributes = svg2paths(path)
    xs, ys, lengths = [], [], []
    for path in paths:
        length = path.length(0, 1.0)
        N = int(length/20) if int(length/20) >= 50 else 50
        ts = np.linspace(0.0, 1.0, N)
        x, y, length = [],[],[]
        prev_t = 0
        for t in ts:
            # length.append(path.length(prev_t, t))
            point = path.point(t)
            x.append(np.real(point))
            y.append(1052.3622 - np.imag(point))  # 1052.36220 - height of A4 page
            prev_t = t

        x = np.array(x)*0.08 + blc[0]
        y = -np.array(y)*0.08 + blc[1]
        length = np.array(length)
        xs.append(x)
        ys.append(y)
        lengths.append(length)

    params_list, start_points = [], []

    order = 3

    for x, y, l in zip(xs, ys, lengths):
        points = np.hstack(
            (-x[np.newaxis].T, y[np.newaxis].T, 325 * np.ones(len(x))[np.newaxis].T, np.zeros(len(x))[np.newaxis].T))
        points2irc(c, points)
        sol = points2irc(c, points)
        params = poly.interpolate(sol)
        params_list.append(params)
        plt.plot(points[:, 0], points[:, 1])
        start_points.append(np.copy(points[0]))

    # plt.show()

    # for start, params in zip(start_points, params_list):
    #     c.move_to_pos([start[0], start[1], 300, start[3]])
    #     prev_a = c.move_to_pos(start)
    #     c.wait_ready(sync=True)
    #     for i in range(len(params)):
    #         c.splinemv(params[i], order=order, min_time=150)
    #     c.wait_ready(sync=True)
    #     c.move_to_pos([start[0], start[1], 300, start[3]])



if __name__ == '__main__':
    robot = robotBosch()
    # delattr(robot, "gripper_init")
    c = Commander(robot)
    c.open_comm('COM3', speed=19200)
    # c.move_to_pos([-300, 220, 20, 0])
    # draw_svg('img/charmander.svg', [-200, 400])
    showBoschRose(c)
    # c.rcon.write('PURGE:\n')
