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

''' Module provides functions for robot control via Mars 8 unit. '''

#  based on code by P. Pisa

import sys
import serial
import time
from robotCRS import *


class Commander:

    def __init__(self, robot, rcon=None):
        self.robot = robot
        self.rcon = rcon #type: serial.Serial
        self.stamp = int(time.time() % 0x7fff)
        self.last_trgt_irc = None
        self.coordmv_commands_to_next_check = 0
        self.coord_axes = None

    def set_rcon(self, rcon):
        if self.rcon is not None:
            self.rcon.close()
            self.rcon = None
        self.rcon = rcon

    def send_cmd(self, cmd):
        ba = bytearray(cmd, 'ascii')
        self.rcon.write(ba)

    def read_resp(self, maxbytes):
        resp = self.rcon.read(maxbytes)
        if resp is None:
            return None
        s = resp.decode("ascii")
        s = s.replace("\r\n", "\n")
        s = s.replace("\r", "\n")
        return s

    def irctoangles(self, a):
        j = np.atleast_2d(a).shape[1]
        n = np.atleast_2d(a).shape[0]
        assert j == self.robot.DOF, 'Wrong number of joints (%d, should be %d).' % (j, self.robot.DOF)

        b = np.divide((a - np.repeat(np.array([self.robot.hhirc]), n, axis=0)),
                        np.repeat(np.array([self.robot.degtoirc]), n, axis=0)) \
            + np.repeat(np.array([self.robot.hhdeg]), n, axis=0)
        if n == 1:
            return b[0]
        else:
            return b

    def anglestoirc(self, a):
        j = np.atleast_2d(a).shape[1]
        n = np.atleast_2d(a).shape[0]
        assert j == self.robot.DOF, 'Wrong number of joints (%d, should be %d).' % (j, self.robot.DOF)

        b = np.multiply((a - np.repeat(np.array([self.robot.hhdeg]), n, axis=0)),
                        np.repeat(np.array([self.robot.degtoirc]), n, axis=0)) \
            + np.repeat(np.array([self.robot.hhirc]), n, axis=0)
        if n == 1:
            return np.rint(b[0])
        else:
            return np.rint(b)

    def degtorad(self, d):
        return d * np.pi / 180.0

    def radtodeg(self, d):
        return d * 180.0 / np.pi

    def init_robot(self):
        self.sync_cmd_fifo()
        if hasattr(self.robot, 'REGPWRON') and self.robot.REGPWRON == 1:
            self.send_cmd('REGPWRON:%i\n'%self.robot.REGPWRON)
            print('Press ARM POWER button,\n')
            if sys.version_info[0] < 3:
                raw_input('press enter to continue...')
            else:
                input('press enter to continue...')
            self.send_cmd('REGPWRFLG:%i\n'%self.robot.REGPWRFLG)

        print('Resetting motors')
        # Purge
        self.send_cmd("PURGE:\n")
        self.send_cmd("STOP:\n")
        s = self.read_resp(2000)
        print(s)

        self.check_ready()
        self.wait_ready()

        self.set_speed_par(self.robot.defaultspeed)
        self.set_acc_par(self.robot.defaultacceleration)

        fields = ['REGME', 'REGCFG', 'REGP', 'REGI', 'REGD']

        for f in fields:
            param_list = getattr(self.robot, f, [])
            if param_list:
                for i in range(self.robot.DOF):
                    if self.robot.activemotors[i]:
                        self.send_cmd('%s%s:%i\n' % (f, self.robot.activemotors[i], param_list[i]))

        if hasattr(self.robot, 'IDLEREL'):
            self.send_cmd('IDLEREL:%i\n'%self.robot.IDLEREL)

        if hasattr(self.robot, 'gripper_init'):
            if self.robot.verbose:
                print('Gripper init.\n')
            robot = self.robot.gripper_init(self)

        if self.robot.description[:3] == 'CRS':
            self.send_cmd('SPDTB:0,300\n')

    def sync_cmd_fifo(self):
        self.stamp = (self.stamp + 1) & 0x7fff
        self.send_cmd('STAMP:%d\n' % self.stamp)
        buf='\n'
        while True:
            buf += self.read_resp(1024)
            i = buf.find('\n' + 'STAMP=')
            if i < 0:
                continue
            s = '%d'%self.stamp
            r = buf[i+7:]
            j = r.find('\n')
            if j < 0:
                continue
            r = r[0:j].strip()
            if r == s:
                break

    def check_ready(self, for_coordmv_queue = False):
        a = int(self.query('ST'))
        s = ''
        if a & 0x8:
            s = 'error, '
        if a & 0x10000:
            s += 'arm power is off, '
        if a & 0x20000:
            s += 'motion stop, '
        if s:
            raise Exception('Check ready: %s.'%s[:-2])
        if for_coordmv_queue:
            return False if a & 0x80 else True
        else:
            return False if a & 0x10 else True

    def set_speed_par(self, params, force=False):
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(params[i]) or params[i] == 0: # relative speed
                    r = np.imag(params[i])
                    if r < 0 or r > 1:
                        raise  Exception('Relative speed %i out of <0;1>'%i)
                    params[i] = round(self.robot.minspeed[i] * (1 - r) + self.robot.maxspeed[i] * r)
                    self.send_cmd('%s%s:%i\n'%('REGMS', self.robot.activemotors[i], params[i]))
                elif not force and (params[i] < self.robot.minspeed[i] or params[i] > self.robot.maxspeed[i]):
                    # speed is not inside lower and upper bound
                    raise Exception('Speed %d is out of bound'%i)
                else: # set the speed
                    self.send_cmd('%s%s:%i\n'%('REGMS', self.robot.activemotors[i], params[i]))

    def set_acc_par(self, params, force=False):
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(params[i]) or params[i] == 0: # relative speed
                    r = np.imag(params[i])
                    if r < 0 or r > 1:
                        raise  Exception('Relative acceleration %i out of <0;1>'%i)
                    params[i] = round(self.robot.minacceleration[i] * (1 - r) + self.robot.maxacceleration[i] * r)
                    self.send_cmd('%s%s:%i\n'%('REGACC', self.robot.activemotors[i], params[i]))
                elif not force and (params[i] < self.robot.minacceleration[i] or params[i] > self.robot.maxacceleration[i]):
                    # speed is not inside lower and upper bound
                    raise Exception('Acceleration %d is out of bound'%i)
                else: # set the speed
                    self.send_cmd('%s%s:%i\n'%('REGACC', self.robot.activemotors[i], params[i]))

    def init_communication(self):
        s = self.read_resp(1024)
        self.send_cmd("\nECHO:0\n")
        self.sync_cmd_fifo()
        s = self.query('VER')
        print('Firmware version : ' + s)

    def set_int_param_for_axes(self, param, val, axes_list=None):
        if axes_list is None:
            axes_list = self.robot.control_axes_list
        valstr = str(int(val))
        for a in axes_list:
            self.send_cmd(param + a + ':' + valstr + '\n')

    def set_max_speed(self, val, axes_list=None):
        self.set_int_param_for_axes(axes_list=axes_list, param='REGMS', val=val)

    def setup_coordmv(self, axes_list=None):
        if axes_list is None:
            axes_list = self.robot.coord_axes
        self.wait_ready()
        axes_coma_list = ','.join(axes_list)
        print(axes_coma_list, axes_list)
        self.send_cmd('COORDGRP:' + axes_coma_list + '\n')
        print('COORDGRP:', axes_coma_list)
        self.wait_ready()
        self.coord_axes = axes_list
        self.last_trgt_irc = None

    def throttle_coordmv(self):
        throttled = False
        if self.coordmv_commands_to_next_check <= 0:
            self.coordmv_commands_to_next_check = 20
        else:
            self.coordmv_commands_to_next_check -= 1
            return
        while not self.check_ready(for_coordmv_queue = True):
            if not throttled:
                print('coordmv queue full - waiting')
            throttled = True
        return throttled

    def coordmv(self, pos, min_time=None, relative=False, disc=5):
        self.throttle_coordmv()
        self.send_cmd('COORDISCONT:%d'%disc + '\n')
        cmd = 'COORDMV' if not relative else 'COORDRELMVT'
        if (min_time is not None) and not relative:
            cmd += 'T'
        cmd += ':'
        if (min_time is not None) or relative:
            if min_time is None:
                min_time=0
            cmd += str(int(round(min_time)))
            if len(pos) > 0:
                cmd += ','
        pos = [int(round(p)) for p in pos]
        cmd += ','.join([str(p) for p in pos])
        print('cmd', cmd)
        self.send_cmd(cmd + '\n')
        if relative:
            if self.last_trgt_irc is None:
                return
            pos = [p + t for p,t in zip(pos, self.last_trgt_irc)]
            # pos = map(int.__add__, pos, self.last_trgt_irc)
        self.last_trgt_irc = pos

    def splinemv(self, param, order=1, min_time=None, disc=5):
        self.throttle_coordmv()
        self.send_cmd('COORDISCONT:%d' % disc + '\n')
        param = [int(round(p)) for p in param]
        cmd = 'COORDSPLINET'
        if min_time is None:
            min_time = 0
        cmd += ':'
        cmd += str(int(round(min_time)))
        cmd += ','
        cmd += str(int(round(order)))
        cmd += ','
        cmd += ','.join([str(p) for p in param])

        self.send_cmd(cmd + '\n')
        return cmd

    def axis_get_pos(self, axis_lst=None):
        if axis_lst is None:
            axis_lst = self.robot.coord_axes
        resp = self.query('COORDAP')
        try:
            resp = np.array(map(int,resp.split(',')))
        except:
            print('Error responce', resp)
        t = resp[0]
        pos = resp[-6:]
        return t, pos

    def hard_home(self, axes_list=None):
        # Hard-home
        if axes_list is None:
            axes_list = self.robot.hh_axes_list

        self.set_speed_par(self.robot.defaultspeed)
        self.set_acc_par(self.robot.defaultacceleration)

        self.last_trgt_irc = None

        for a in axes_list:
            self.send_cmd('HH' + a + ':\n')
            self.wait_ready()

    def soft_home(self, axes_list=None):
        if axes_list is None:
            axes_list = self.robot.hh_axes_list
        self.setup_coordmv()
        if hasattr(self.robot, 'shdeg'):
            self.coordmv(self.anglestoirc(np.array(self.robot.shdeg)))
        else:
            self.coordmv(self.anglestoirc(np.array(self.robot.hhdeg)))
        self.wait_ready()

    def query(self, query):
        buf = '\n'
        self.send_cmd('\n' + query + '?\n')
        while True:
            buf += self.read_resp(1024)
            i = buf.find('\n' + query + '=')
            if i < 0:
                continue
            j = buf[i+1:].find('\n')
            if j != -1:
                break
        if buf[i+1+j-1] == '\r':
            j -= 1
        res = buf[i+2+len(query):i+1+j]
        return res

    def command(self, command):
        self.send_cmd(command + ':\n')

    def move_to_pos(self, pos, prev_pos=None, relative=False, move=True):
        a = self.robot.ikt(self.robot, pos)
        num = None
        min_dist = float('Inf')

        if hasattr(self.robot, 'shdeg'):
            prev_pos = self.anglestoirc(np.array(self.robot.shdeg)) if prev_pos is None else prev_pos
        else:
            prev_pos = self.anglestoirc(np.array(self.robot.hhdeg)) if prev_pos is None else prev_pos
        for i in range(len(a)):
            irc = self.anglestoirc(a[i])
            validm = irc > self.robot.bound[0]
            validp = irc < self.robot.bound[1]
            valid = np.logical_and(validm, validp)
            if np.all(valid):
                dist = np.linalg.norm(np.array(irc) - prev_pos)
                if dist < min_dist:
                    min_dist = dist
                    num = i
        if num == None:
            return None
        irc = self.anglestoirc(a[num])
        if move:
            if self.last_trgt_irc is None:
                relative=False
            if relative:
                prev_irc = self.last_trgt_irc
                irc = list(irc - prev_irc)
            self.coordmv(irc, relative=relative)
        return irc

    def wait_ready(self, sync=False):
        buf = '\n'
        if sync:
            self.sync_cmd_fifo()
            print('Synchronized!')
        self.send_cmd("\nR:\n")
        while True:
            buf += self.read_resp(1024)
            if buf.find('\nR!') >= 0:
                return True
            if buf.find('\nFAIL!') >= 0:
                return False

    def wait_gripper_ready(self):
        if not hasattr(self.robot, 'gripper_ax'):
            raise Exception('This robot has no gripper_ax defined.')

        self.send_cmd('\nR%s:\n'%self.robot.gripper_ax)
        self.rcon.timeout = 2

        s = self.read_resp(1024)
        self.rcon.timeout = 0.01
        if s.find('R%s!\r\n' % self.robot.gripper_ax) >= 0:
            last = float('inf')
            while True:
                self.send_cmd('AP%s?\n'%self.robot.gripper_ax)
                s = self.read_resp(1024)

                if s.find('\nFAIL!') >= 0:
                    raise Exception('Command \'AP\' returned \'FAIL!\n')
                ifs = s.find('AP%s=' % self.robot.gripper_ax)
                if ifs >= 0:
                    ifl = s.find('\r\n')
                    p = float(s[ifs+4:ifl])
                if abs(last - p) < self.robot.gripper_poll_diff:
                    break
                last = p
            time.sleep(self.robot.gripper_poll_time/100)
            return
        if s.find('\nFAIL!') >= 0:
            self.wait_ready()
            raise Exception('Command \'R:%s\' returned \'FAIL!\''%self.robot.gripper_ax)

    def open_comm(self, tty_dev, speed=19200):

        print("Opening %s ...\n" % tty_dev)
        ser = serial.Serial(tty_dev,
                            baudrate=speed,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            rtscts=True,
                            timeout=0.01)

        self.set_rcon(ser)
        self.init_communication()

    def release(self):
        self.send_cmd("RELEASE:\n")

    def init(self, **kwargs):
        self.init_robot()
        reg_type = kwargs.get('reg_type', None)
        max_speed = kwargs.get('max_speed', None)
        hard_home = kwargs.get('hard_home', True)

        if reg_type is not None:
            self.send_cmd("RELEASE:\n")
            self.set_int_param_for_axes(param='REGTYPE', val=reg_type)

        if max_speed is not None:
            self.set_max_speed(val=max_speed)

        if hard_home:
            print("Running home")
            self.hard_home()
            self.soft_home()
            print("Hard and soft home done!")

    def reset_motors(self):
        self.send_cmd("PURGE:\n")
