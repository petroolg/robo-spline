import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from PIL import Image, BmpImagePlugin

from svgpathtools import svg2paths, wsvg, Path

l_stick_mm = 220.0 # length of stick
l_led_diode_mm = 1.5 # height of single diode


def preprocess_sequence(time_pos_rot, freq):
    t = time_pos_rot[0][0]
    end_time = time_pos_rot[-1][0]
    t_step = 1 / freq
    pos_rot = []
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
    pos_rot = np.array(pos_rot)
    plt.plot(pos_rot[:,0])
    plt.plot(pos_rot[:,1])
    plt.plot(pos_rot[:,2])
    plt.show()
    return pos_rot


def extract_seq(file, time_pos_rot, im_width_mm, freq):

    input_im = Image.open(file)  # type: BmpImagePlugin.BmpImageFile
    input_im.load()
    im_width_px, im_height_px = input_im.size
    size_ratio = im_width_px / im_width_mm
    l_stick_px = size_ratio * l_stick_mm  # length of stick in px
    px_rad = int(size_ratio * l_led_diode_mm)
    px_rad = px_rad + 1 if px_rad % 2 == 0 else px_rad
    x = np.linspace(-(px_rad-1)/2, (px_rad-1)/2, px_rad)
    mask_w = px_rad*px_rad
    px_mask = np.reshape(np.meshgrid(x,x), (2,int(mask_w))).T
    pos_rot = preprocess_sequence(time_pos_rot, freq)
    out_im = Image.new("RGB", (72, len(pos_rot)), (0, 0, 0))
    i = 0
    plt.plot(pos_rot[:,0],pos_rot[:,1])
    plt.show()
    for pr in pos_rot:
        for j in range(72):
            # pixel center
            pc = np.rint((pr[0] + l_stick_px * j / 71.0 * np.cos((pr[2]+90)/180.0*np.pi),
                          im_height_px - (pr[1] + l_stick_px * j / 71.0 * np.sin((pr[2]+90)/180.0*np.pi))))
            mask = px_mask + pc
            p = np.array([0,0,0])
            for r in mask:
                try:
                    p += input_im.getpixel((r[0],r[1]))
                except IndexError:
                    continue
            out_im.putpixel((j, i), tuple(p/mask_w))
        i += 1

    out_im.save('led_sequence.bmp')

    rev_trans(pos_rot, l_stick_px, px_rad)

def rev_trans(pos_rot, im_size, l_stick_px, px_rad):
    input_im = Image.open('led_sequence.bmp')  # type: BmpImagePlugin.BmpImageFile
    input_im.load()
    im_width_px, im_height_px = input_im.size
    out_im = Image.new("RGB", im_size[1], (0, 0, 0))

    x = np.linspace(-(px_rad - 1) / 2, (px_rad - 1) / 2, px_rad, dtype=int)
    mask_w = px_rad * px_rad
    px_mask = np.reshape(np.meshgrid(x, x), (2, int(mask_w))).T

    for i, pr in enumerate(pos_rot):
        for j in range(72):
            # pixel center
            coord_x = int(round(pr[0] + l_stick_px * j / 71.0  * np.cos((pr[2] + 90 )/180.0*np.pi)))
            coord_y = im_size[1]-int(round(pr[1] +l_stick_px * j / 71.0  * np.sin((pr[2] + 90) / 180.0 * np.pi)))
            mask = px_mask + [coord_x, coord_y]
            for px in mask:
                try:
                    out_im.putpixel(tuple(px), input_im.getpixel((j, i)))
                except IndexError:
                    continue
    out_im.save('control.bmp')


def plot_deriv(xs, ys, ds):
    angles = []
    for x, y, d in zip(xs, ys, ds):
        # y = tg(dy/dx)
        i = np.imag(d)
        r = np.real(d)
        # k = i/(-r*np.sign(r))
        plt.plot([x, x+10*(-i*np.sign(i))],[y, y+10*(r*np.sign(r))])
        angles.append(np.arctan2((r*np.sign(r)), (-i*np.sign(i))))
    return angles

def run_avg(vec, n):
    p = []
    for i in range(len(vec)):
        p.append(np.mean(vec[max(0, i - n/2) : min(i+n/2, len(vec)-1)]))
    return np.array(p)

def svg_to_coord(path, bot_left_corn, width_mm):
    paths, attributes = svg2paths(path)
    ts = np.linspace(0.0, 1.0, 100)
    x, y, d = [], [], []

    redpath = paths[0]  # type: Path
    for t in ts:
        point = redpath.point(t)
        der = redpath.derivative(t)
        x.append(np.real(point))
        y.append(1052.3622-np.imag(point)) # 1052.36220 - height of A4 page
        d.append(der)
    x = np.array(x)
    y = np.array(y)
    image_bl_corn = [min(x), min(y)]
    im_w_pt = (max(x) - min(x))
    im_h_pt = (max(y) - min(y))
    im_ratio = im_h_pt / im_w_pt  # height/width
    heigh_mm = im_ratio * width_mm
    x = (x - min(x)) / im_w_pt * width_mm + bot_left_corn[0]
    y = (y - min(y)) / im_h_pt * heigh_mm + bot_left_corn[1]

    plt.plot(x, y)
    angles = plot_deriv(x,y,d)
    angles = np.rad2deg(np.array(angles) - np.pi/2)
    angles = run_avg(angles, 20)[np.newaxis].T
    trajectory = np.hstack((550 * np.ones((len(x), 1)), x[np.newaxis].T, y[np.newaxis].T, np.zeros((len(x), 2)), angles))
    np.save('demo/trajectory.npy', trajectory)
    plt.show()
    plt.plot(angles)
    plt.show()

    return image_bl_corn, [im_w_pt, im_h_pt]


def coord_to_image(path, bot_left_corn, im_size, im_bl_corn):
    t = path[:,0][np.newaxis].T
    x = path[:,2][np.newaxis].T
    y = path[:,3][np.newaxis].T
    r = (path[:,6][np.newaxis].T - 90.0) / 180.0 * np.pi
    x = x - bot_left_corn[0]
    y = y - bot_left_corn[1]
    x = (x - min(x)) / (max(x) - min(x)) * im_size[0] + im_bl_corn[0]
    y = (y - min(y)) / (max(y) - min(y)) * im_size[1] + im_bl_corn[1]
    tpr = np.hstack((t, x, y, r))
    return tpr


