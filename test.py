#!/usr/bin/env python

from CRS_commander import Commander
from robotCRS97 import robCRS97
import argparse
import b_spline
import p_spline
import poly
import numpy as np
from robCRSgripper import robCRSgripper

if __name__ == '__main__':

    help_msg = 'SYNOPSIS: CRS_commander.py [-l /dev/ttyXXX]'

    parser = argparse.ArgumentParser(description='CRS robot commander')
    parser.add_argument('-s', '--skip-setup', dest='skip_setup', action='store_true',
                        default=True, help='skip hard-home inicialization of robot')
    parser.add_argument('-d', '--tty-device', dest='tty_dev', type=str,
                        default='COM3', help='tty line/device to robot')  #'/dev/ttyUSB0'
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
                        default=20, help='step of circle sampling in degs')
    parser.add_argument('-g', '--graph', dest='graph', action='store_true',
                        default=True, help='show graph')
    parser.add_argument('-sp', '--spline', dest='spline',  type=str,
                        default='b-spline',
                        help='type of spline to use for interpolation\n{\'poly\', \'b-spline\', \'p-spline\'}')
    parser.add_argument('-o', '--order', dest='order', type=int,
                        default=2, help='order of splines')

    args = parser.parse_args()

    tty_dev = args.tty_dev
    skip_setup = args.skip_setup
    max_speed = args.max_speed
    reg_type = args.reg_type
    action = args.action
    x0 = args.x0
    y0 = args.y0
    z0 = args.z0
    graph = True#args.graph
    radius = args.r
    step = args.step
    spline = args.spline
    order = 2

    robot = robCRS97()
    c = Commander(robot)
    # c.open_comm(tty_dev)

    if not skip_setup or action == 'home':
        c.init(reg_type=reg_type, max_speed=max_speed)

    if graph or action == 'move':
        sol = c.circle(x0, y0, z0, radius, step, move=False)
        # sol = c.line([500, 300, 500, 0, 0, 0], [500, 300, 550, 0, 0, 0], step=5, move=False)

    if graph or action == 'move':
        if spline == 'poly':
            poly.interpolate(sol, graph=graph)
            params = np.load('param.npy')
            order = 3
        if spline == 'b-spline':
            b_spline.interpolate(sol, order=order,graph=graph)
            params = np.load('param_b_spline_%d.npy' % order)
        if spline == 'p-spline':
            num_segments = int(len(sol)/2)
            poly_deg = 2
            penalty_order = 2
            lambda_ = 0.1
            p_spline.interpolate(sol, num_segments, poly_deg, penalty_order, lambda_, graph=True)
            params = np.load('param_p_spline_%d.npy' % order)

    if action == 'move':
        pos = [x0, y0+radius, z0, 0, 0, 0]
        prev_a = c.move_to_pos(pos)
        c.wait_ready(sync=True)
        for i in range(len(params)):
            c.splinemv( params[i], order=order)
        c.wait_ready(sync=True)

    if action == 'circle':
        c.circle(x0, y0, z0, radius, step=1)

    if action == 'grip':
        robCRSgripper(c, -0.9)
        c.wait_ready()
        # c.wait_gripper_ready()
        # c.release()

    if action == 'release':
        c.rcon.write("RELEASE:\n")