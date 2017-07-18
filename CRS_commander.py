#!/usr/bin/env python

''' Module provides functions for robot control via Mars 8 unit. '''

# (c) 2009-05, V. Smutny ???
# (c) 2010-01-27, Martin Matousek

import sys
import serial
import time
import numpy as np
import argparse
from robCRSikt import robCRSikt
from robotCRS97 import Robot

class Commander:

    def __init__(self, rcon=None):
        self.robot = Robot()
        self.rcon = rcon
        self.last_trgt_irc = None

    def set_rcon(self, rcon):
        if self.rcon is not None:
            self.rcon.close()
            self.rcon = None
        self.rcon = rcon

    def irctoangles(self, irc):
        itc = np.array(irc)
        angles = np.divide(irc - self.robot.hhirc, self.robot.degtoirc) + self.robot.hhdeg
        return angles

    def anglestoirc(self, angles):
        irc = np.multiply((angles - self.robot.hhdeg), self.robot.degtoirc) + self.robot.hhirc;
        return irc

    def degtorad(self, d):
        return d * np.pi / 180.0

    def radtodeg(self, d):
        return d * 180.0 / np.pi

    def init_robot(self):
        print('Resetting motors')
        # Purge
        self.rcon.write("PURGE:\n")
        s = self.rcon.read(2000)
        print(s)

        self.check_ready()
        self.wait_ready()

        self.set_speeds(self.robot.defaultspeed)
        self.set_acc(self.robot.defaultacceleration)

        fields = ['REGME', 'REGCFG', 'REGP', 'REGI', 'REGD']

        for f in fields:
            param_list = []
            exec ('param_list = self.robot.' + f)
            if param_list:
                for i in range(self.robot.DOF):
                    if self.robot.activemotors[i]:
                        self.rcon.write('%s%s:%i' % (f, self.robot.activemotors[i], param_list[i]))

        if hasattr(self.robot, 'IDLEREL'):
            self.rcon.write('IDLEREL:%i'%self.robot.IDLEREL)

        if hasattr(self.robot, 'gripper_init'):
            if self.robot.verbose:
                print('Gripper init.\n')
            robot = self.robot.gripper_init(self)

        if self.robot.description[:3] == 'CRS':
            self.rcon.write('SPDTB:0,300')

    def check_ready(self):
        a = int(self.query('ST'))
        s = ''
        if a & 0x8:
            s = 'error, '
        if a & 0x10000:
            s = s + 'arm power is off, '
        if a & 0x20000:
            s = s + 'motion stop, '
        if s:
            raise Exception('Check ready:' + s[:-2] + '.')
        return False if a & 0x4000 else True

    def set_speeds(self, speeds):
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(speeds[i]) or speeds[i] == 0: # relative speed
                    r = np.imag(speeds[i])
                    if r < 0 or r > 1:
                        raise  Exception('Relative speed %i out of <0;1>', i)
                    speeds[i] = round(self.robot.minspeed[i] * (1 - r) + self.robot.maxspeed[i] * r)
                    self.rcon.write('REGMS%s:%i'%(self.robot.activemotors[i], speeds[i]))
                elif speeds[i] < self.robot.minspeed[i] or speeds[i] > self.robot.maxspeed[i]:
                    # speed is not inside lower and upper bound
                    raise Exception('Speed %i out of bound', i)
                else: # set the speed
                    self.rcon.write('REGMS%s:%i'%(self.robot.activemotors[i], speeds[i]))

    def set_acc(self, acc):
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(acc[i]) or acc[i] == 0: # relative speed
                    r = np.imag(acc[i])
                    if r < 0 or r > 1:
                        raise Exception('Relative acceleration %i out of <0;1>', i)
                    acc[i] = round(self.robot.minacceleration[i] * (1 - r) + self.robot.maxacceleration[i] * r)
                    self.rcon.write('REGACC%s:%i'%(self.robot.activemotors[i], acc[i]))
                elif acc[i] < self.robot.minacceleration[i] or acc[i] > self.robot.maxacceleration[i]:
                    # speed is not inside lower and upper bound
                    raise Exception('Acceleration %i out of bound', i)
                else: # set the speed
                    self.rcon.write('REGACC%s:%i'%(self.robot.activemotors[i], acc[i]))

    def init_communication(self):
        s = self.rcon.read(1024)

        self.rcon.write("InvBuff\n")
        s = self.rcon.read(1024)

        self.rcon.write("ECHO:0\n")
        s = self.rcon.read(1024)
        self.rcon.write("VER?\n")
        s = self.rcon.read(1024)
        print(s)

    def set_int_param_for_axes(self, param, val, axes_list=None):
        if axes_list is None:
            axes_list = self.robot.control_axes_list
        valstr = str(int(val))
        for a in axes_list:
            self.rcon.write(param + a + ':' + valstr + '\n')

    def set_max_speed(self, val, axes_list=None):
        self.set_int_param_for_axes(axes_list=axes_list, param='REGMS', val=val)

    def setup_coordmv(self, axes_list=None):
        if axes_list is None:
            axes_list = self.coord_axes
        self.wait_ready()
        axes_coma_list = ','.join(axes_list)
        print(axes_coma_list, axes_list)
        self.rcon.write('COORDGRP:' + axes_coma_list + '\n')
        print('COORDGRP:', axes_coma_list)
        self.wait_ready()
        self.coord_axes = axes_list

    def coordmv(self, pos, min_time=None, relative=False):

        cmd = 'COORDMV' if not relative else 'COORDRELMVT'
        if (min_time is not None) and not relative:
            cmd += 'T'
        cmd += ':'
        if (min_time is not None) or relative:
            if min_time is None:
                min_time=1
            cmd += str(int(round(min_time)))
            if len(pos) > 0:
                cmd += ','
        cmd += ','.join([str(int(round(p))) for p in pos])
        print('cmd', cmd)
        self.rcon.write(cmd + '\n')

    def splinemv(self, param, order=1, min_time=None):
        cmd = 'COORDSPLINET'
        if min_time is None:
            min_time = 0
        cmd += ':'
        cmd += str(int(round(min_time)))
        cmd += ','
        cmd += str(int(round(order)))
        cmd += ','
        cmd += ','.join([str(int(round(p))) for p in param])
        # cmd += ','.join([str(int(round(p))) + ',0' for p in param])

        print('cmd', cmd)
        self.rcon.write(cmd + '\n')

    def hard_home(self, axes_list=None):
        # Hard-home
        if axes_list is None:
            axes_list = self.robot.hh_axes_list
        for a in axes_list:
            self.rcon.write('HH' + a + ':\n')
        self.wait_ready()
        for a in axes_list:
            self.rcon.write('HH' + a + ':\n')
            self.wait_ready()
        self.last_trgt_irc=None

    def query(self, q):
        buf = '\n'
        # while len(self.rcon.read(1024)) != 0:
        #     pass
        self.rcon.write('\n' + q + '?\n')
        while True:
            buf += self.rcon.read(1024)
            i = buf.find('\n' + q + '=')
            if i<0:
                continue
            if buf[i+1:].find('\n') != -1:
                break
        return buf[i+2+len(q):]

    def move_to_pos(self, pos, prev_pos = None, relative=False):
        a = c.robot.ikt(c.robot, pos)
        valid_lst = []
        num = -1
        irc = []
        for i in range(len(a)):
            irc = c.robot.anglestoirc(a[i])
            validm = irc > self.robot.bound[0]
            validp = irc < self.robot.bound[1]
            valid = np.logical_and(validm, validp)
            if np.all(valid):
                num = i
                valid_lst.append(i)
                break

        if prev_pos is not None:
            min_dist = 10000
            for i, s in enumerate(a):
                dist = np.linalg.norm(np.array(a[i]) - prev_pos)
                if dist < min_dist and i in valid_lst:
                    min_dist = dist
                    num = i
            irc = c.robot.anglestoirc(a[num])


        # print('Number of solution:', num)
        # print('Solution:', a[num])
        # print('Valid list:', valid_lst)
        if self.last_trgt_irc is None:
            relative=False
        prev_irc = self.last_trgt_irc
        while True:
            st_s = self.query('ST') #type: str
            print 'ST=' + st_s
            try:
                st = int(st_s.strip('\n\r '))
                print 'ST num=' + str(st)
            except:
                continue
            if (st & 128) == 0:
                break

        self.last_trgt_irc = [int(round(p)) for p in irc]
        if relative:
            irc=list(irc - prev_irc)
        if relative:
            self.splinemv(irc)
        else:
            self.coordmv(irc, relative=relative)
        # wait_ready(c.rcon)
        return a[num]

    def wait_ready(self):
        # s = self.rcon.read(1024)
        # self.rcon.write("\nR:\n")
        while 1:
            self.rcon.write("R:\n")
            s = self.rcon.read(1024)
            print 'Read:' + s + '\n'
            if (s[-4:-2] == "R!"):
                break
            # time.sleep(1)

    def wait_gripper_ready(self):
        if not hasattr(self.robot, 'gripper_ax'):
            raise Exception('This robot has no gripper_ax defined.')

        self.rcon.write('\nR:%s\n'%self.robot.gripper_ax)
        s = self.rcon.read(1024)
        if s == 'R:%s!\r\n' % self.robot.gripper_ax:
            last = float('inf')
            while True:
                self.rcon.write('AP%s?'%self.robot.gripper_ax)
                s = self.rcon.read

                if s == 'FAIL!\r\n':
                    raise Exception('Command \'AP\' returned \'FAIL!\n')

                if s[:4] == 'AP%s=' % self.robot.gripper_ax:
                    p = float(s[5:-2])
                if abs(last - p) < self.robot.gripper_poll_diff:
                    break
                last = p
            time.sleep(self.robot.gripper_poll_time)
            return
        if s == 'FAIL!\r\n':
            self.wait_ready()
            raise Exception( 'Command \'R:%s\' returned \'FAIL!\''%self.robot.gripper_ax)


def circle(c #type: Commander
            , x=500, y0=250, z0=500, r=50):
    c.setup_coordmv()
    pos = [x, y0+r, z0, 0, 0, 0]
    prev_a = c.move_to_pos(pos)
    for i in range(360):
        y = y0 + r*np.cos(i/180.0*np.pi)
        z = z0 + r*np.sin(i/180.0*np.pi)
        pos = [x, y, z, 0, 0, 0]
        # print('pos', pos)
        # print('i', i)
        prev_a = c.move_to_pos(pos, prev_a, relative=True)


if __name__ == '__main__':

    help_msg = 'SYNOPSIS: CRS_commander.py [-l /dev/ttyXXX]'

    parser = argparse.ArgumentParser(description='CRS robot commander')
    parser.add_argument('-s', '--skip-setup', dest='skip_setup', action='store_true',
                        default=False, help='skip hard-home inicialization of robot')
    parser.add_argument('-d', '--tty-device', dest='tty_dev', type=str,
                        default='COM3', help='tty line/device to robot')  # /dev/ttyUSB0
    parser.add_argument('-m', '--max-speed', dest='max_speed', type=int,
                        default=None, help='maximal motion speed')
    parser.add_argument('-t', '--reg-type', dest='reg_type', type=int,
                        default=8, help='controller type selection')
    parser.add_argument('--reg-s1', dest='reg_s1', type=int,
                        default=None, help='controller S1 constant')
    parser.add_argument('--reg-s2', dest='reg_s2', type=int,
                        default=None, help='controller S2 constant')
    parser.add_argument('-T', '--reg-preset', dest='reg_preset', action='store_true',
                        default=False, help='skip hard-home inicialization of robot')
    parser.add_argument('-p', '--target-position', dest='target_position', type=int,
                        default=None, help='target position')
    parser.add_argument('-a', '--action', dest='action', type=str,
                        default=None, help='action to run')

    args = parser.parse_args()

    tty_dev = args.tty_dev
    skip_setup = True
    max_speed = args.max_speed
    reg_type = args.reg_type
    reg_s1 = args.reg_s1
    reg_s2 = args.reg_s2
    action = 'poly'

    c = Commander()

    if True:
        print("Opening %s ...\n" % tty_dev)
        ser = serial.Serial(tty_dev,
                            baudrate=19200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            rtscts=True,
                            timeout=0.01)

        c.set_rcon(ser)
        c.init_communication()

    if reg_type is not None:
        c.rcon.write("RELEASE:\n")
        c.set_int_param_for_axes(param='REGTYPE', val=reg_type)

    c.init_robot()

    if not skip_setup or (action == 'home'):
        # c.set_max_speed(val=1000)

        print("Running hard home")
        c.hard_home()
        print("Hard home done!")

    if max_speed is not None:
        c.set_max_speed(val=max_speed)

    if action == 'release':
        c.rcon.write("RELEASE:\n")

    if action == 'circle':
        circle(c)

    if action == 'poly':
        c.setup_coordmv()
        params = np.load('param.npy')
        pos = [500, 300, 500, 0, 0, 0]
        prev_a = c.move_to_pos(pos)
        c.wait_ready()
        irc = [0]*18
        for i in range(6):
            # irc[0:-2:3] = c.robot.anglestoirc(params[i][0:-2:3])
            # irc[1:-1:3] = c.robot.anglestoirc(params[i][1:-1:3])
            # irc[2::3] = c.robot.anglestoirc(params[i][2::3])
            # irc = [params[i][a]*256 for a in range(18)]
            c.splinemv( params[i], order=3)
        c.wait_ready()

