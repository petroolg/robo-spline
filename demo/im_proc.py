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

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from PIL import Image, BmpImagePlugin

from svgpathtools import svg2paths, wsvg, Path

l_stick_mm = 192.0 # length of stick
l_led_diode_mm = 1.5 # height of single diode


def preprocess_sequence(time_pos_rot, freq, orig_x, orig_y):
    t = time_pos_rot[0][0]
    start_time = time_pos_rot[0][0]
    end_time = time_pos_rot[-1][0]
    t_step = 1 / freq
    pos_rot = []
    print('total time of show:', end_time - t)
    while len(time_pos_rot) > 1 and t < end_time:

        if t == time_pos_rot[0][0]:
            pos_rot.append(time_pos_rot[0][1:])
        else:
            ti = (t - time_pos_rot[0][0])/(time_pos_rot[1][0] - time_pos_rot[0][0])
            pos = time_pos_rot[0][1:3] + (time_pos_rot[1][1:3] - time_pos_rot[0][1:3])*ti
            rot = time_pos_rot[0][3] + (time_pos_rot[1][3] - time_pos_rot[0][3])*ti
            pos_rot.append(np.hstack((pos, rot)))

        t += t_step
        while len(time_pos_rot) > 1 and t > time_pos_rot[1][0]:
            time_pos_rot = np.delete(time_pos_rot,0,0)
    print('total time of show:', t-start_time)
    pos_rot = np.array(pos_rot)
    pos_rot[:, 2] = np.deg2rad(pos_rot[:, 2] + 90.0)
    plt.plot(pos_rot[:,0],pos_rot[:,1], label='computed')
    plt.plot(orig_x, orig_y, label='original')
    for x,y,a in pos_rot:
        plt.plot([x, x - 50 * np.cos(a)], [y, y - 50 * np.sin(a)])
    # plt.waitforbuttonpress()
    # plt.close()
    plt.legend()
    plt.show()
    plt.plot(pos_rot[:,2])
    # plt.waitforbuttonpress()
    # plt.close()
    plt.show()

    return pos_rot


def extract_seq(file, time_pos_rot, size_ratio, freq, orig_x, orig_y):

    input_im = Image.open(file)  # type: BmpImagePlugin.BmpImageFile
    input_im.load()
    im_width_px, im_height_px = input_im.size
    l_stick_px = size_ratio * l_stick_mm  # length of stick in pt
    px_rad = int(size_ratio * l_led_diode_mm)
    px_rad = px_rad + 1 if px_rad % 2 == 0 else px_rad
    x = np.linspace(-(px_rad-1)/2, (px_rad-1)/2, px_rad)
    mask_w = px_rad*px_rad
    px_mask = np.reshape(np.meshgrid(x,x), (2,int(mask_w))).T
    pos_rot = preprocess_sequence(time_pos_rot, freq, orig_x, orig_y)
    out_im = Image.new("RGB", (72, len(pos_rot)), (0, 0, 0))
    i = 0
    for pr in pos_rot:
        for j in range(72):
            # pixel center
            pc = np.rint((pr[0] + l_stick_px * j / 71.0 * np.cos(pr[2]),
                          im_height_px - (pr[1] + l_stick_px * j / 71.0 * np.sin(pr[2]))))
            mask = px_mask + pc
            p = np.array([0,0,0])
            for r in mask:
                try:
                    p += input_im.getpixel((r[0],r[1]))
                except IndexError:
                    continue
            out_im.putpixel((j, i), tuple(p/mask_w))
        i += 1

    out_im.save('demo/generated_files/led_sequence.bmp')

    rev_trans(pos_rot, (int(im_width_px), int(im_height_px)), l_stick_px, px_rad)

def rev_trans(pos_rot, im_size, l_stick_px, px_rad):
    input_im = Image.open('demo/generated_files/led_sequence.bmp')  # type: BmpImagePlugin.BmpImageFile
    input_im.load()
    im_width_px, im_height_px = input_im.size
    out_im = Image.new("RGB", im_size, (0, 0, 0))

    x = np.linspace(-(px_rad - 1) / 2, (px_rad - 1) / 2, px_rad, dtype=int)
    mask_w = px_rad * px_rad
    px_mask = np.reshape(np.meshgrid(x, x), (2, int(mask_w))).T

    for i, pr in enumerate(pos_rot):
        for j in range(72):
            # pixel center
            coord_x = int(round(pr[0] + l_stick_px * j / 71.0  * np.cos(pr[2])))
            coord_y = im_size[1]-int(round(pr[1] +l_stick_px * j / 71.0  * np.sin(pr[2])))
            mask = px_mask + [coord_x, coord_y]
            for px in mask:
                try:
                    out_im.putpixel(tuple(px), input_im.getpixel((j, i)))
                except IndexError:
                    continue
    out_im.save('demo/generated_files/rec_img.bmp')


def plot_deriv(xs, ys):
    n_x, n_y, angles = [], [], []
    my = min(ys)
    im_h = max(ys) - min(ys)
    for x, y in zip(xs, ys):
        # y = tg(dy/dx)
        # i = np.imag(d)
        # r = np.real(d)
        # a = 90 * (1 - (y - my) / im_h) + 90
        a = 90.0
        n_x.append(x - 50.0 * np.cos(np.deg2rad(a)))
        n_y.append(y - 50.0 * np.sin(np.deg2rad(a)))
        # k = i/(-r*np.sign(r))
        plt.plot([x, x-50*np.cos(np.deg2rad(a))],[y, y-50*np.sin(np.deg2rad(a))])
        angles.append([a])
    return np.array(n_x), np.array(n_y), angles

def run_avg(vec, n):
    p = []
    for i in range(len(vec)):
        p.append(np.mean(vec[max(0, i - n/2) : min(i+n/2, len(vec)-1)]))
    return np.array(p)

def svg_to_coord(path, bot_left_corn, width_mm):
    paths, attributes = svg2paths(path)
    ts = np.linspace(0.0, 1.0, 100)
    x, y, d, lengths = [], [], [], []
    prev_t = 0
    redpath = paths[0]  # type: Path
    for t in ts:
        lengths.append(redpath.length(prev_t,t))
        point = redpath.point(t)
        # der = redpath.derivative(t)
        x.append(np.real(point))
        y.append(1052.3622-np.imag(point)) # 1052.36220 - height of A4 page
        # d.append(der)
        prev_t = t
    x = np.array(x)
    y = np.array(y)
    orig_x = x.copy()/1.242
    orig_y = y.copy()/1.242
    image_bl_corn = [min(x)/1.242, min(y)/1.242]
    im_w_pt = (max(x) - min(x))/1.242
    im_h_pt = (max(y) - min(y))/1.242
    lengths = np.array(lengths) / im_w_pt * width_mm
    im_ratio = im_h_pt / im_w_pt  # height/width
    heigh_mm = im_ratio * width_mm
    x = (x - min(x)) / (1.242*im_w_pt) * width_mm + bot_left_corn[0]
    y = (y - min(y)) / (1.242*im_h_pt) * heigh_mm + bot_left_corn[1]

    x, y, angles = plot_deriv(x,y)
    angles = np.array(angles) - 90.0
    # trajectory for toolpoint
    trajectory = np.hstack((550 * np.ones((len(x), 1)), x[np.newaxis].T, y[np.newaxis].T, np.zeros((len(x), 2)), angles))
    np.save('demo/trajectory/trajectory.npy', trajectory)
    np.save('demo/trajectory/times.npy', np.rint(lengths))
    print(lengths)
    plt.plot(x, y)
    plt.waitforbuttonpress()
    plt.close()
    plt.plot(angles)
    plt.waitforbuttonpress()
    plt.close()
    plt.show()

    return image_bl_corn, [im_w_pt, im_h_pt], im_w_pt/width_mm, orig_x, orig_y


def coord_to_image(path, bot_left_corn, im_size, im_bl_corn, label):
    r = path[:,6][np.newaxis].T
    t = path[:,0][np.newaxis].T
    x = path[:,2][np.newaxis].T + 50.0*np.cos(np.deg2rad(r+90.0))
    y = path[:,3][np.newaxis].T + 50.0*np.sin(np.deg2rad(r+90.0))
    x = x - bot_left_corn[0]
    y = y - bot_left_corn[1]
    x = (x - min(x)) / (max(x) - min(x)) * im_size[0] + im_bl_corn[0]
    y = (y - min(y)) / (max(y) - min(y)) * im_size[1] + im_bl_corn[1]

    tpr = np.hstack((t, x, y, r))
    return tpr


