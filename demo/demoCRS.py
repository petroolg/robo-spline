import time
import numpy as np
from demo.im_proc import *
import poly
from robCRSdkt import robCRSdkt
import sched
import serial
from serial import Serial

FREC = 1.0 / 8000.0 * 1e6

def open_comm(tty_dev, speed=19200):
    print("Opening %s ...\n" % tty_dev)
    ser = serial.Serial(tty_dev,
                        baudrate=speed,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        rtscts=True,
                        timeout=0.01)
    return ser

def start_show(rcon):
    rcon.write('GH:30\n')
    time.sleep(1.0)
    rcon.write('GH:327\n')

def showCRS(c):
    # self.defaultspeed = np.array([30, 8, 20, 30, 30, 55]) * 256
    # self.defaultacceleration = np.rint(
    #     np.array([(30.0 / 400.0), (8.0 / 400.0), (20.0 / 400.0), (30.0 / 400.0), 1.0 / 2.0, 3.0 / 5.0]) * 256.0)

    if c.rcon:
        speed = np.array([120, 32, 80, 120, 120, 200]) * 256.0
        acceleration = np.rint(
             np.array([(15.0 / 400.0), (4.0 / 400.0), (10.0 / 400.0), (15.0 / 400.0), 0.5 / 2.0, 1.5 / 5.0]) * 256.0)

        c.set_speed_par(speed,force=True)
        c.set_acc_par(acceleration,force=True)
        # c.setup_coordmv('ABCDEFH')

        c.rcon.write('REGMEH:32730\n')
        c.rcon.write('REGIH:0\n')
        c.rcon.write('REGDH:0\n')
        c.rcon.write('REGPH:100\n')
        c.rcon.write('GH:0\n')
        time.sleep(1.0)
        c.rcon.write('GH:327\n')
    #
    # # convert SVG to path
    # blcorn, im_size = svg_to_coord('demo/traj.svg', [-240.0, 200.0], 480.0)
    # points = np.load('demo/trajectory.npy')
    # sol = np.empty((1,6))
    # prev_a = None
    # for p in points:
    #     # print( cy + np.cos(a)*r, cz + np.sin(a)*r)
    #     prev_a = c.move_to_pos(p, prev_a, move=False)
    #     sol = np.append(sol, [prev_a], axis=0)
    # sol = np.delete(sol, 0, 0)
    #
    # # interpolate points
    # poly.interpolate(sol)
    # params = np.load('params/param_poly.npy')
    # order = 3
    #
    # print('length of params: %d'%(len(params)))
    #
    # # Gathering exact sequence of place-time
    # c.move_to_pos(points[0])
    # c.wait_ready(sync=True)
    # time_pos_irc = []
    #
    # show_start = time.time() + 8.0
    # c.splinemv(np.zeros(18), order=3, min_time=8000)
    # preload_s = min(120,len(params))
    # for i in range(preload_s):
    #     c.splinemv(params[i], order=order)
    # wait = show_start - time.time()
    # if wait > 0:
    #     time.sleep(wait)
    # start_show(c.rcon)
    # print 'Waiting time%f'%wait
    # for i in range(preload_s, len(params)):
    #     c.splinemv(params[i], order=order)
    #
    #
    # # Gather place-time irc
    # pos = sol[0]
    # end_pos = sol[-1]
    # d = max(pos-end_pos)
    # while d > 20.0:
    #     pos = c.axis_get_pos()
    #     time_pos_irc.append((time.time(), pos))
    #     d = max(pos-end_pos)
    #
    # print('Number of gathered points:%d'%(len(time_pos_irc)))
    #
    # c.wait_ready()
    # print('End of motion%f' % (time.time() - show_start))
    #
    # # Convert place-time irc to time-coordinates tuple
    # coord = []
    # for tpi in time_pos_irc:
    #     ang = c.irctoangles(tpi[1])
    #     coord.append(np.hstack((tpi[0], robCRSdkt(c.robot,ang))))
    # coord = np.array(coord)
    #
    # # Convert space coordinates to image coordinates
    # time_pos_rot = coord_to_image(coord,[-240.0, 200.0], im_size, blcorn)
    #
    # plt.plot(time_pos_rot[:,1], time_pos_rot[:,2])
    # plt.show()
    #
    # np.save('time_pos_rot.npy', time_pos_rot)
    time_pos_rot = np.load('time_pos_rot.npy')

    # Extract sequence of pixels for stick from image
    extract_seq('bk/img/the_starry_night-wallpaper-1366x768.bmp', time_pos_rot, 480.0, FREC)