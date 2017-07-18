#!/usr/bin/env python

from CRS_commander import Commander
import argparse
from poly import circle_points, interpolate
import numpy as np

if __name__ == '__main__':

    help_msg = 'SYNOPSIS: CRS_commander.py [-l /dev/ttyXXX]'

    parser = argparse.ArgumentParser(description='CRS robot commander')
    parser.add_argument('-s', '--skip-setup', dest='skip_setup', action='store_true',
                        default=False, help='skip hard-home inicialization of robot')
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

    args = parser.parse_args()

    tty_dev = args.tty_dev
    skip_setup = args.skip_setup
    max_speed = args.max_speed
    reg_type = args.reg_type
    action = 'graph' #args.action
    x0 = args.x0
    y0 = args.y0
    z0 = args.z0
    radius = args.r
    step = args.step

    c = Commander()
    c.open_comm(tty_dev)

    if not skip_setup or action == 'home':
        c.init(reg_type=reg_type, max_speed=max_speed)

    if action == 'graph':
        sol = circle_points(c, x0, y0, z0, radius, step)
        interpolate(sol, graph=True)

    if action == 'circle':
        c.circle(c, x0, y0, z0, radius)

    if action == 'poly':
        sol = circle_points(c, x0, y0, z0, radius, step)
        interpolate(sol, graph=False)

        params = np.load('param.npy')
        pos = [x0, y0+radius, z0, 0, 0, 0]
        prev_a = c.move_to_pos(pos)
        c.wait_ready()
        for i in range(len(params)):
            c.splinemv( params[i], order=3)
        c.wait_ready()



