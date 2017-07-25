#!/usr/bin/env python

from CRS_commander import Commander
from robotCRS97 import robCRS97
import argparse
import b_spline
import poly
import numpy as np
from robCRSgripper import robCRSgripper

if __name__ == '__main__':

    help_msg = 'SYNOPSIS: CRS_commander.py [-l /dev/ttyXXX]'

    parser = argparse.ArgumentParser(description='CRS robot commander')
    parser.add_argument('-s', '--skip-setup', dest='skip_setup', action='store_true',
                        default=True, help='skip hard-home inicialization of robot')
    parser.add_argument('-d', '--tty-device', dest='tty_dev', type=str,
                        default='/dev/ttyUSB0', help='tty line/device to robot')
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
                        default=50, help='radius of circle')
    parser.add_argument('-st', '--step', dest='step', type=int,
                        default=5, help='step of circle sampling in degs')
    parser.add_argument('-g', '--graph', dest='graph', type=str,
                        default=True, help='graph only')

    args = parser.parse_args()

    tty_dev = args.tty_dev
    skip_setup = args.skip_setup
    max_speed = args.max_speed
    reg_type = args.reg_type
    action = 'b-spline' #args.action
    x0 = args.x0
    y0 = args.y0
    z0 = args.z0
    graph = args.graph
    radius = args.r
    step = 90#args.step

    robot = robCRS97()
    c = Commander(robot)
    # c.open_comm(tty_dev)

    if not skip_setup or action == 'home':
        c.init(reg_type=reg_type, max_speed=max_speed)

    if graph:
        move=False
        sol = c.circle(x0, y0, z0, radius, step, move)
        if action=='poly':
            poly.interpolate(sol, graph=True)
        if action=='b-spline':
            b_spline.interpolate(sol, graph=True)

    if action == 'circle':
        c.circle(x0, y0, z0, radius, step=1)

    if action == 'poly' and not graph:
        sol = c.circle(x0, y0, z0, radius, step, move=False)
        poly.interpolate(sol, graph=False)

        params = np.load('param.npy')
        pos = [x0, y0+radius, z0, 0, 0, 0]
        prev_a = c.move_to_pos(pos)
        c.wait_ready(sync=True)
        for i in range(len(params)):
            c.splinemv( params[i], order=3)
        c.wait_ready(sync=True)


    if action == 'b-spline_2' and not graph:
        sol = c.circle(x0, y0, z0, radius, step, move=False)
        b_spline.interpolate(sol, graph=False)

        params = np.load('param_spline.npy')
        pos = [x0, y0+radius, z0, 0, 0, 0]
        prev_a = c.move_to_pos(pos)
        c.wait_ready(sync=True)
        for i in range(len(params)):
            c.splinemv( params[i], order=2)
        c.wait_ready(sync=True)

    if action == 'grip':
        robCRSgripper(c, -0.9)
        c.wait_ready()
        # c.wait_gripper_ready()
        # c.release()