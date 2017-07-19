#!/usr/bin/env python

''' Module provides functions for robot control via Mars 8 unit. '''

#  based on code by P. Pisa

import serial
import time
from robotCRS97 import *

class Commander:

    def __init__(self, robot, rcon=None):
        self.robot = robot
        self.rcon = rcon #type: serial.Serial
        self.stamp = int(time.time()%0x7fff)
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
        self.sync_cmd_fifo()
        print('Resetting motors')
        # Purge
        self.rcon.write("PURGE:\n")
        s = self.rcon.read(2000)
        print(s)

        self.check_ready()
        self.wait_ready()

        self.set_motion_par('REGMS', self.robot.defaultspeed)
        self.set_motion_par('REGACC', self.robot.defaultacceleration)

        fields = ['REGME', 'REGCFG', 'REGP', 'REGI', 'REGD']

        for f in fields:
            param_list = getattr(self.robot, f, [])
            if param_list:
                for i in range(self.robot.DOF):
                    if self.robot.activemotors[i]:
                        self.rcon.write('%s%s:%i\n' % (f, self.robot.activemotors[i], param_list[i]))

        if hasattr(self.robot, 'IDLEREL'):
            self.rcon.write('IDLEREL:%i\n'%self.robot.IDLEREL)

        if hasattr(self.robot, 'gripper_init'):
            if self.robot.verbose:
                print('Gripper init.\n')
            robot = self.robot.gripper_init(self)

        if self.robot.description[:3] == 'CRS':
            self.rcon.write('SPDTB:0,300\n')

    def sync_cmd_fifo(self):
        self.stamp = (self.stamp + 1) & 0x7fff
        self.rcon.write('STAMP:%d\n' % self.stamp)
        buf='\n'
        while True:
            buf += self.rcon.read(1024)
            i = buf.find('\n' + 'STAMP=')
            if i < 0:
                continue
            if buf[i+1:].find('%d'%self.stamp) != -1:
                break

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
        return False if a & 0x10 else True

    def set_motion_par(self, command, params):
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(params[i]) or params[i] == 0: # relative speed
                    r = np.imag(params[i])
                    if r < 0 or r > 1:
                        raise  Exception('Relative speed %i out of <0;1>', i)
                    params[i] = round(self.robot.minspeed[i] * (1 - r) + self.robot.maxspeed[i] * r)
                    self.rcon.write('%s%s:%i\n'%(command, self.robot.activemotors[i], params[i]))
                elif params[i] < self.robot.minspeed[i] or params[i] > self.robot.maxspeed[i]:
                    # speed is not inside lower and upper bound
                    raise Exception('Speed %i out of bound', i)
                else: # set the speed
                    self.rcon.write('%s%s:%i\n'%(command, self.robot.activemotors[i], params[i]))

    def init_communication(self):
        s = self.rcon.read(1024)
        self.rcon.write("ECHO:0\n")
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
            axes_list = self.robot.coord_axes
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
                min_time=0
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

        self.rcon.write('HH' + axes_list[1] + ':\n')
        self.wait_ready()

        self.rcon.write('HH' + axes_list[0] + ':\n')
        self.rcon.write('HH' + axes_list[2] + ':\n')
        self.wait_ready()

        self.rcon.write('HH' + axes_list[3] + ':\n')
        self.rcon.write('HH' + axes_list[4] + ':\n')
        self.rcon.write('HH' + axes_list[5] + ':\n')
        self.wait_ready()

        self.setup_coordmv()
        self.coordmv(self.anglestoirc(np.array(self.robot.shdeg)))
        self.wait_ready()

        self.last_trgt_irc=None

    def query(self, query):
        buf = '\n'
        while len(self.rcon.read(1024)) != 0:
            pass
        self.rcon.write(query + '?\n')
        while True:
            buf += self.rcon.read(1024)
            i = buf.find('\n' + query + '=')
            if i < 0:
                continue
            if buf[i+1:].find('\n') != -1:
                break
        return buf[i+2+len(query):]

    def command(self, command):
        self.rcon.write(command + ':\n')

    def move_to_pos(self, pos, prev_pos = None, relative=False):
        a = self.robot.ikt(self.robot, pos)
        num = -1
        irc = []
        min_dist = 10000
        for i in range(len(a)):
            irc = self.robot.anglestoirc(a[i])
            validm = irc > self.robot.bound[0]
            validp = irc < self.robot.bound[1]
            valid = np.logical_and(validm, validp)
            if np.all(valid):
                if prev_pos is not None:
                    dist = np.linalg.norm(np.array(a[i]) - prev_pos)
                    if dist < min_dist:
                        min_dist = dist
                        num = i
                else:
                    num = i
                    break

        irc = self.robot.anglestoirc(a[num])
        # print('Number of solution:', num)
        # print('Solution:', a[num])
        # print('Valid list:', valid_lst)
        if self.last_trgt_irc is None:
            relative=False
        prev_irc = self.last_trgt_irc

        self.last_trgt_irc = [int(round(p)) for p in irc]
        if relative:
            irc = list(irc - prev_irc)
        if relative:
            self.splinemv(irc)
        else:
            self.coordmv(irc, relative=relative)
        return a[num]

    def wait_ready(self, sync=False):
        buf = '\n'
        if sync:
            self.sync_cmd_fifo()
            print('Synchronized!')
        self.rcon.write("\nR:\n")
        while True:
            buf += self.rcon.read(1024)
            if buf.find('\nR!') >= 0:
                return True
            if buf.find('\nFAIL!') >= 0:
                return False

    def wait_gripper_ready(self):
        if not hasattr(self.robot, 'gripper_ax'):
            raise Exception('This robot has no gripper_ax defined.')

        self.rcon.write('\nR%s:\n'%self.robot.gripper_ax)
        self.rcon.timeout = 2

        s = self.rcon.read(1024)
        self.rcon.timeout = 0.01
        if s.find('R%s!\r\n' % self.robot.gripper_ax) >= 0:
            last = float('inf')
            while True:
                self.rcon.write('AP%s?\n'%self.robot.gripper_ax)
                s = self.rcon.read(1024)

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
            raise Exception( 'Command \'R:%s\' returned \'FAIL!\''%self.robot.gripper_ax)

    def open_comm(self, tty_dev):

        print("Opening %s ...\n" % tty_dev)
        ser = serial.Serial(tty_dev,
                            baudrate=19200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            rtscts=True,
                            timeout=0.01)

        self.set_rcon(ser)
        self.init_communication()

    def release(self):
        self.rcon.write("RELEASE:\n")

    def init(self, **kwargs):
        self.init_robot()
        reg_type = kwargs.get('reg_type', None)
        max_speed = kwargs.get('max_speed', None)
        if reg_type is not None:
            self.rcon.write("RELEASE:\n")
            self.set_int_param_for_axes(param='REGTYPE', val=reg_type)

        if max_speed is not None:
            self.set_max_speed(val=max_speed)

        print("Running hard home")
        self.hard_home()
        print("Hard home done!")

    def circle(self, x=500, y0=250, z0=500, r=50):
        pos = [x, y0+r, z0, 0, 0, 0]
        prev_a = self.move_to_pos(pos)
        for i in range(360):
            y = y0 + r*np.cos(i/180.0*np.pi)
            z = z0 + r*np.sin(i/180.0*np.pi)
            pos = [x, y, z, 0, 0, 0]
            # print('pos', pos)
            # print('i', i)
            prev_a = self.move_to_pos(pos, prev_a, relative=True)