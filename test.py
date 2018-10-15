#!/usr/bin/env python

# Coordinated Spline Motion and Robot Control Project
# 
# Copyright (c) 2017 Olga Petrova <olga.petrova@cvut.cz>
# Advisor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
# FEE CTU Prague, Czech Republic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# In 2017, project funded by PiKRON s.r.o. http://www.pikron.com/


# Script provides functionality for CRS robot initialisation, example trajectories and trajectory visualisation.
# Show graph of circle trajectory, don't move: python test.py -s -a graph
# Move along circle trajectory point-to-point: python test.py -a circle
# Move along circle trajectory using interpolated trajectory: python test.py -a move

import argparse
import os
import numpy as np

from CRS_commander import Commander
#from demo.im_proc import *
from graph import Graph
from interpolation import *
from robCRSgripper import robCRSgripper
from robotBosch import robotBosch
from robotCRS import robCRS93, robCRS97


def circle(commander, x=500, y0=250, z0=500, r=50, step=5, move=True):
    """
    Circle trajectory for CRS robot.
    :param commander: Robot commander
    :param x: X coordinate of circle plane
    :param y0: Y coordinate of starting point
    :param z0: Z coordinate of starting point
    :param r: radius of circle
    :param step: step of trajectory discretisation
    :param move: boolean, whether to perform movement or not
    :return: starting point in world coordinates, parameters of trajectory
    """
    pos = [x, y0 + r, z0, 0, 0, 0]
    prev_a = commander.move_to_pos(pos, relative=False, move=move)
    sol = np.zeros((1, 6))
    rng = int(360 / step)
    for i in range(rng + 1):
        y = y0 + r * np.cos((i * step) / 180.0 * np.pi)
        z = z0 + r * np.sin((i * step) / 180.0 * np.pi)
        pos = [x, y, z, 0, 0, 0]
        prev_a = commander.move_to_pos(pos, prev_a, relative=False, move=move)
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)
    return [x, y0 + r, z0, 0, 0, 0], sol


def line(commander, x0, x1, step=5, move=True):
    """
    Line trajectory for CRS robot.
    :param commander: Robot commander
    :param x0: starting point of trajectory
    :param x1: end point of trajectory
    :param step: tep of trajectory discretisation
    :param move: boolean, whether to perform movement or not
    :return: starting point in world coordinates, parameters of trajectory
    """
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
                        default=False, help='skip hard-home inicialization of robot')
    parser.add_argument('-d', '--tty-device', dest='tty_dev', type=str,
                        default='/dev/ttyUSB0', help='tty line/device to robot')
    parser.add_argument('-a', '--action', dest='action', type=str,
                        default='graph', help='action to run, possible actions:\n \
                                            {home - homing of the robot,\n \
                                             graph - draw graph of interpolated trajectory without moving,\n \
                                             move - move along interpolated trajectory,\n \
                                             circle - move along sampled trajectory simply point to point,\n \
                                             grip - close gripper,\n \
                                             purge - purge errors on motors}')
    parser.add_argument('-r', '--robot', dest='robot', type=str,
                        default='CRS97', help='type of robot\n{CRS97, CRS93, Bosch}')
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
    parser.add_argument('-R', '--radius', dest='r', type=int,
                        default=100, help='radius of circle')
    parser.add_argument('-st', '--step', dest='step', type=int,
                        default=10, help='step of circle sampling in degs')
    parser.add_argument('-sp', '--spline', dest='spline',  type=str,
                        default='poly',
                        help='type of spline to use for interpolation\n{poly, b-spline, p-spline}')
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
    radius = args.r
    step = args.step
    spline = args.spline
    order = args.order
    rob = args.robot

    robot = None
    if rob == 'CRS97':
        robot = robCRS97()
    if rob == 'CRS93':
        robot = robCRS93()
    if rob == 'Bosch':
        robot = robotBosch()

    c = Commander(robot)
    c.open_comm(tty_dev, speed=19200)

    if not skip_setup or action == 'home':
        c.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)

    if action == 'graph' or action == 'move':
        start_point, sol = circle(c, x0, y0, z0, radius, step, move=False)

        if start_point is None or sol is None:
            raise Exception('Unfeasible trajectory.')

    if action == 'move':
        if not os.path.isdir('params'):
            os.mkdir('params')
        if spline == 'poly':
            params = poly.interpolate(sol)
            order = 3
        if spline == 'b-spline':
            params = b_spline.interpolate(sol, order=order)
        if spline == 'p-spline':
            num_segments = int(len(sol)/3)
            poly_deg = order
            penalty_order = 2
            lambda_ = 0.1
            params = p_spline.interpolate(sol, num_segments, poly_deg, penalty_order, lambda_)

        prev_a = c.move_to_pos(start_point)
        c.wait_ready(sync=True)
        for i in range(len(params)):
            c.splinemv(params[i], order=order)
        c.wait_ready(sync=True)

    if action == 'circle':
        circle(c, x0, y0, z0, radius, step=1)

    if action == 'grip':
        robCRSgripper(c, 0.9)
        c.wait_ready()

    if action == 'graph':
        e = Graph(sol)
        e.show_gui()

    if action == 'purge':
        c.send_cmd("PURGE:\n")
