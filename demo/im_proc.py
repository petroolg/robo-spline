import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from PIL import Image, BmpImagePlugin

from svgpathtools import svg2paths, wsvg

l_stick_mm = 220.0 # length of stick
l_led_diode_mm = 1.5 # height of single diode


def preprocess_sequence(time_pos_rot, freq):
    t = time_pos_rot[0][0]
    end_time = time_pos_rot[-1][0]
    t_step = 1 / freq
    pos_rot = []
    while time_pos_rot and t < end_time:

        if t == time_pos_rot[0][0]:
            pos_rot.append((time_pos_rot[0][1], time_pos_rot[0][2]))
        else:
            ti = (t - time_pos_rot[0][0])/(time_pos_rot[1][0] - time_pos_rot[0][0])
            pos = time_pos_rot[0][1] + time_pos_rot[1][1]*ti
            rot = time_pos_rot[0][2] + time_pos_rot[1][2]*ti
            pos_rot.append((pos, rot))

        t += t_step
        while time_pos_rot and t > time_pos_rot[1][0]:
            time_pos_rot.pop(0)
    return pos_rot


def extract_seq(file, time_pos_rot, im_height_mm, freq):

    input_im = Image.open(file)  # type: BmpImagePlugin.BmpImageFile
    input_im.load()
    _, im_height_px = input_im.size
    size_ratio = im_height_px / im_height_mm
    l_stick_px = size_ratio * l_stick_mm  # length of stick in px
    px_rad = size_ratio * l_led_diode_mm
    px_rad = px_rad + 1 if px_rad % 2 == 0 else px_rad
    x = np.linspace(-(px_rad-1)/2, (px_rad-1)/2, px_rad)
    mask_w = px_rad*px_rad
    px_mask = np.reshape(np.meshgrid(x,x), (2,int(mask_w))).T
    pos_rot = preprocess_sequence(time_pos_rot, freq)
    out_im = Image.new("RGB", (72, len(pos_rot)), (0, 0, 0))
    i = 0
    for pr in pos_rot:
        for j in range(72):
            # pixel center
            pc = (pr[0][0] + l_stick_px * j / 71.0 * np.cos(pr[1]),
                 pr[0][1] + l_stick_px * j / 71.0 * np.sin(pr[1]))
            mask = px_mask + pc
            p = np.array([0,0,0])
            for r in mask:
                p += input_im.getpixel((r[0],r[1]))
            out_im.putpixel(tuple(p/mask_w), (j, i))
        i += 1


def svg_to_coord(path, bot_left_corn, im_size, width_mm):
    paths, attributes = svg2paths('traj.svg')
    ts = np.linspace(0.0, 1.0, 100)
    x, y = [], []
    im_ratio = im_size[1]/im_size[0]  # height/width
    heigh_mm = im_ratio * width_mm
    redpath = paths[0]
    for t in ts:
        point = redpath.point(t)
        x.append(np.real(point))
        y.append(-np.imag(point))

    x = (np.array(x)-min(x)) / (max(x) - min(x)) * width_mm + bot_left_corn[0]
    y = (np.array(y)-min(y)) / (max(y) - min(y)) * heigh_mm + bot_left_corn[1]
    trajectory = np.hstack((550 * np.ones((len(x), 1)), x[np.newaxis].T, y[np.newaxis].T, np.zeros((len(x), 3))))
    np.save('trajectory.npy', trajectory)
    plt.plot(x, y)
    plt.show()

if __name__ == '__main__':
    svg_to_coord('traj.svg', [-240.0, 100.0], [2126.0, 1417.0], 480.0)



