#!/usr/bin/env python

import os
import time
from CRS_commander import Commander
from robotCRS97 import robCRS97
import argparse
import b_spline
import p_spline
import poly
import numpy as np
from graph import Graph
from robCRSgripper import robCRSgripper


def circle(commander, x=500, y0=250, z0=500, r=50, step=5, move=True):
    pos = [x, y0 + r, z0, 0, 0, 0]
    prev_a = commander.move_to_pos(pos, relative=False, move=move)
    sol = np.zeros((1, 6))
    rng = 360 / step
    for i in range(rng + 1):
        y = y0 + r * np.cos((i * step) / 180.0 * np.pi)
        z = z0 + r * np.sin((i * step) / 180.0 * np.pi)
        pos = [x, y, z, 0, 0, 0]
        prev_a = commander.move_to_pos(pos, prev_a, relative=False, move=move)
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)
    return [x, y0 + r, z0, 0, 0, 0],sol

def show():

    cy = 0
    cz = 100
    r = 650
    angles = np.linspace(1.2217, np.pi - 1.2217, 50)
    angles_grip = np.linspace(-60, 60, 50)
    sol = np.zeros((1,6))
    # start = [550, cy + np.cos(angles[0]) * r, cz + np.sin(angles[0]) * r, 0, 0, angles_grip[0]]

    # points = np.loadtxt('bk\\traj.txt')
    points = np.load('bk\\trajectory.npy')
    points[:,-1] *= -1

    for p in points:
        # print( cy + np.cos(a)*r, cz + np.sin(a)*r)
        prev_a = c.move_to_pos(p, None, relative=False,move=False)
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)

    return points[0], sol


def line(commander, x0, x1, step=5, move=True):
    x = x0
    prev_x = commander.move_to_pos(x, relative=False, move=move)
    sol = np.zeros((1, 6))
    rng = int(np.linalg.norm(np.array(x0) - np.array(x1)) / step)
    normal = (np.array(x1) - np.array(x0)) / np.linalg.norm(np.array(x0) - np.array(x1))
    for i in range(rng):
        x = x + normal * step
        prev_x = commander.move_to_pos(x, prev_x, relative=False, move=move)
        sol = np.append(sol, [prev_x], axis=0)
    sol = np.delete(sol, 0, 0)
    return x0, sol


if __name__ == '__main__':

    help_msg = 'SYNOPSIS: CRS_commander.py [-l /dev/ttyXXX]'

    parser = argparse.ArgumentParser(description='CRS robot commander')
    parser.add_argument('-s', '--skip-setup', dest='skip_setup', action='store_true',
                        default=True, help='skip hard-home inicialization of robot')
    parser.add_argument('-d', '--tty-device', dest='tty_dev', type=str,
                        default='COM3', help='tty line/device to robot')  # /dev/ttyUSB0
    parser.add_argument('-a', '--action', dest='action', type=str,
                        default='home', help='action to run')
    parser.add_argument('-m', '--max-speed', dest='max_speed', type=int,
                        default=None, help='maximal motion speed')
    parser.add_argument('-t', '--reg-type', dest='reg_type', type=int,
                        default=None, help='controller type selection')
    parser.add_argument('-p', '--target-position', dest='target_position', type=int,
                        default=None, help='target position')

    parser.add_argument('-x', '--x-coord', dest='x0', type=int,
                        default=500, help='x coord. of circle center')
    parser.add_argument('-y', '--y-coord', dest='y0', type=int,
                        default=250, help='y coord. of circle center')
    parser.add_argument('-z', '--z-coord', dest='z0', type=int,
                        default=500, help='z coord. of circle center')
    parser.add_argument('-r', '--radius', dest='r', type=int,
                        default=100, help='radius of circle')
    parser.add_argument('-st', '--step', dest='step', type=int,
                        default=10, help='step of circle sampling in degs')
    parser.add_argument('-sp', '--spline', dest='spline',  type=str,
                        default='poly',
                        help='type of spline to use for interpolation\n{\'poly\', \'b-spline\', \'p-spline\'}')
    parser.add_argument('-o', '--order', dest='order', type=int,
                        default=2, help='order of splines')

    args = parser.parse_args()

    tty_dev = args.tty_dev
    skip_setup = args.skip_setup
    max_speed = args.max_speed
    reg_type = args.reg_type
    action = 'move'#args.action
    x0 = args.x0
    y0 = args.y0
    z0 = args.z0
    radius = args.r
    step = args.step
    spline = args.spline
    order = args.order

    robot = robCRS97()
    c = Commander(robot)
    c.open_comm(tty_dev)

    if not skip_setup or action == 'home':
        c.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)

    if action == 'graph' or action == 'move':
        # start_point, sol = circle(c, x0, y0, z0, radius, step, move=False)
        start_point, sol = show()
        # start_point, sol = c.line([500, 300, 500, 0, 0, 0], [500, 300, 550, 0, 0, 0], step=5, move=False)

    if not os.path.isdir('params'):
        os.mkdir('params')

    if action == 'move':
        if not os.path.isdir('params'):
            os.mkdir('params')
        if spline == 'poly':
            poly.interpolate(sol)
            params = np.load('params/param_poly.npy')
            order = 3
        if spline == 'b-spline':
            b_spline.interpolate(sol, order=order)
            params = np.load('params/param_b_spline_%d.npy' % order)
        if spline == 'p-spline':
            num_segments = int(len(sol)/3)
            poly_deg = order
            penalty_order = 2
            lambda_ = 0.1
            p_spline.interpolate(sol, num_segments, poly_deg, penalty_order, lambda_)
            params = np.load('params/param_p_spline_%d.npy' % order)

    if action == 'move':
        prev_a = c.move_to_pos(start_point)
        c.wait_ready(sync=True)
        for j in range(1):
            for i in range(len(params)):
                # print('\n' + 'planned' + str(sol[i + 1].astype(int)))
                c.splinemv(params[i], order=order)
            # c.wait_ready(sync=True)

    if action == 'circle':
        circle(c, x0, y0, z0, radius, step=1)

    if action == 'grip':
        robCRSgripper(c, -0.9)
        c.wait_ready()
        # c.wait_gripper_ready()
        # c.release()

    if action == 'purge':
        c.rcon.write("PURGE:\n")

    if action == 'graph':
        e = Graph(sol)
        e.show_gui()

    if action == 'search':
        y = [-80, 80]
        z = [300, 300]
        ys = np.linspace(y[0], y[1], 50, dtype=int)
        zs = np.linspace(z[0], z[1], 10, dtype=int)
        prev_a = None
        points = np.load('bk\\trajectory.npy')
        # for yi in ys:
        #     for zi in zs:
        #         prev_a = c.move_to_pos([550, yi, zi, 0,0,0], prev_a, move=True)
        for p in points:
            prev_a = c.move_to_pos(p, prev_a, move=True)

    if action == 'blink':
        c.rcon.write('RELEASE:\n')
        c.rcon.write('PURGE:\n')
        c.rcon.write('REGMEH:32730\n')
        c.rcon.write('REGIH:0\n')
        c.rcon.write('REGDH:0\n')
        c.rcon.write('REGPH:100\n')
        c.rcon.write('GH:0\n')
        time.sleep(3.0)
        c.rcon.write('GH:327\n')
        # time.sleep(5.0)
        # c.rcon.write('GH:30\n')
        # time.sleep(1.0)
        # c.rcon.write('GH:327\n')

    if action == 'blink_off':
        c.rcon.write('GH:0\n')