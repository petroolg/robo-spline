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
    # time.sleep(1.0)
    # rcon.write('GH:327\n')

def gather_irc(c, show_start, sol):
    # Gather place-time irc
    time_pos_irc = []
    pos = sol[0]
    end_pos = sol[-1]
    d = max(pos-end_pos)
    done = False
    start = time.time()
    while d > 30.0:

        t, pos = c.axis_get_pos()
        time_pos_irc.append((t, pos))
        d = max(pos-end_pos)
        if not done and time.time() - start > 60:
            c.rcon.write('GH:327\n')
            done = True
        # print d

    print('Number of gathered points:%d'%(len(time_pos_irc)))
    c.wait_ready()
    print('End of motion, elapsed time: %f' % (time.time() - show_start))

    # Convert place-time irc to time-coordinates tuple
    gain = 0
    coord = [np.hstack((gain, robCRSdkt(c.robot, c.irctoangles(time_pos_irc[0][1]))))]
    for i in range(1, len(time_pos_irc)-1):
        ang = c.irctoangles(time_pos_irc[i][1])
        diff = time_pos_irc[i][0] - time_pos_irc[i-1][0]
        if diff < 0:
            diff += 0xffff
        gain += diff
        # print('diff', diff)
        coord.append(np.hstack((gain/1000.0, robCRSdkt(c.robot,ang))))
    np.save('demo/trajectory/gathered_points.npy', np.array(coord))
    return np.array(coord)

def set_show_params(c):
    speed = np.array([180, 48, 120, 180, 180, 300]) * 256.0
    acceleration = np.rint(
        np.array([(15.0 / 400.0), (4.0 / 400.0), (10.0 / 400.0), (15.0 / 400.0), 0.5 / 2.0, 1.5 / 5.0]) * 256.0)

    c.set_speed_par(speed, force=True)
    c.set_acc_par(acceleration, force=True)
    # c.setup_coordmv('ABCDEFH')

    c.rcon.write('REGMEH:32730\n')
    c.rcon.write('REGIH:0\n')
    c.rcon.write('REGDH:0\n')
    c.rcon.write('REGPH:100\n')
    c.rcon.write('GH:0\n')
    time.sleep(1.0)
    c.rcon.write('GH:327\n')

def coord_to_irc(c, points):
    sol = np.empty((1, 6))
    prev_a = None
    for p in points:
        # print( cy + np.cos(a)*r, cz + np.sin(a)*r)
        prev_a = c.move_to_pos(p, prev_a, move=False)
        sol = np.append(sol, [prev_a], axis=0)
    sol = np.delete(sol, 0, 0)
    return sol

def showCRS(c):
    print('demo started')
    if c.rcon:
        set_show_params(c)
        c.check_ready()
    print('set parameters')
    # convert SVG to path
    blcorn, im_size, size_ratio, orig_x, orig_y = svg_to_coord('demo/trajectory/traj2.svg', [-240.0, 300.0], 480.0)
    points = np.load('demo/trajectory/trajectory.npy')
    sol = coord_to_irc(c, points)

    # interpolate points
    poly.interpolate(sol)
    params = np.load('params/param_poly.npy')
    order = 3

    print('length of params: %d'%(len(params)))

    # Gathering exact sequence of place-time
    c.move_to_pos(points[0])
    c.wait_ready(sync=True)

    show_start = time.time() + 6.7
    c.splinemv(np.zeros(18), order=3, min_time=7000)
    preload_s = min(120,len(params))
    times = np.load('demo/trajectory/times.npy')
    for i in range(preload_s):
        c.splinemv(params[i], order=order, min_time=80)
    wait = show_start - time.time()
    if wait > 0:
        time.sleep(wait)
    start_show(c.rcon)
    # print 'Waiting time%f'%(wait)
    for i in range(preload_s, len(params)):
        c.splinemv(params[i], order=order,min_time=80)

    coord_real = gather_irc(c, show_start, sol)
    c.wait_ready()
    el_time = time.time() - show_start
    times = np.cumsum(np.ones_like(times)*0.08)[np.newaxis].T
    times = times / times[-1] * el_time
    coord_theor = np.hstack((times,points))

    # Convert space coordinates to image coordinates
    coord_real = np.load('demo/trajectory/gathered_points.npy')
    # time_pos_rot_theor = coord_to_image(coord_theor,[-240.0, 200.0], im_size, blcorn,'Theoretical')
    time_pos_rot_real = coord_to_image(coord_real, [-240.0, 200.0], im_size, blcorn,'Real')

    plt.plot(time_pos_rot_real[:, 1], time_pos_rot_real[:, 2],label='real raw')
    # plt.plot(time_pos_rot_theor[:,1], time_pos_rot_theor[:,2], label='theoretical')
    plt.legend()
    plt.waitforbuttonpress()
    plt.close()

    np.save('demo/trajectory/time_pos_rot.npy', time_pos_rot_real)
    time_pos_rot = np.load('demo/trajectory/time_pos_rot.npy')

    # Extract sequence of pixels for stick from image
    # extract_seq('demo/img/STRIPEMAIN_900x.bmp', time_pos_rot, size_ratio, FREC, orig_x, orig_y)
    extract_seq('demo/img/the_starry_night-wallpaper-1366x768.bmp', time_pos_rot, size_ratio, FREC, orig_x, orig_y)