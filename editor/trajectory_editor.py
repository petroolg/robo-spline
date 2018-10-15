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
from CRS_commander import Commander
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib
from matplotlib.figure import Figure


class Segment:

    def val(self, t): raise NotImplementedError

    @property
    def end(self):
        if hasattr(self, 'end_'):
            return self.end_
        else:
            self.end_ = self.val(1.0)
        return self.end_

    @property
    def length(self):
        if hasattr(self, 'l_'):
            return self.l_

        l, t, delta = 0.0, 0.0, 0.1
        while t < 1.0:
            while True:
                step = self.val(t + delta) - self.val(t)
                step_len = step[0] * step[0] + step[1] * step[1] + step[2] * step[2]
                if step_len > 4.0:
                    delta /= 2
                else:
                    break
            t += delta
            l += step_len
        self.l_ = l
        return self.l_

class LinSeg(Segment):

    def __init__(self, start, end):
        self.start = start
        self.end_ = end
        self.l = np.linalg.norm(end-start)
        self.vec = (end-start)

    def val(self, t):
        return self.start + t * self.vec

class SplineSeg(Segment):
    def __init__(self, start, cx, cy, cz, order=2):
        if order != 2 and order != 3:
            raise ValueError("Constructor supports only 2nd and 3rd order polynomials.")
        if not (len(cx) == len(cy) == len(cz) == order):
            raise ValueError('Number of parameters must be equal to order of polynomial '
                             'cx: %d, cy: %d, cz: %d, order = %d'%(len(cx), len(cy), len(cz), order))

        self.start = start
        self.cx = cx
        self.cy = cy
        self.cz = cz
        self.ord = order

    def val(self, t):
        if self.ord == 2:
            t = np.array([t ** 2, t])
        if self.ord == 3:
            t = np.array([t ** 3, t ** 2, t])

        return self.start + np.array([t.dot(self.cx), t.dot(self.cy), t.dot(self.cz)])

class CircSeg(Segment):

    def __init__(self, center, axis, r, start, angle):
        self.c = center
        self.start = start
        self.angle = angle
        self.r = r
        self.l_ = angle * 2 * np.pi * r

        self.axis = axis / np.linalg.norm(axis)
        self.a = center - start
        self.a /= np.linalg.norm(self.a)
        self.b = np.cross(self.a, self.axis)
        self.b /= np.linalg.norm(self.b)

    def val(self, t):
        return self.c + self.r * np.cos(self.angle * t) * self.a + self.r * np.sin(self.angle * t) * self.b

class Editor:

    def __init__(self):
        # self.c = commander
        self.segments = []
        self.lines = None
        self.selected_lines = None
        self.normal_selected_color = np.array([[0, 0, 1, 1.0], [1, 0, 0, 1.0]])

        self.figure = Figure()
        self.axes = self.figure.gca(projection='3d')

        self.axes.mouse_init()

        # self.figure.canvas.mpl_connect('button_press_event', self)
        # self.figure.canvas.mpl_connect('pick_event', self)
        self.axes.set_xlabel('X')
        self.axes.set_xlim3d(-10, 10)
        self.axes.set_ylabel('Y')
        self.axes.set_ylim3d(-10, 10)
        self.axes.set_zlabel('Z')
        self.axes.set_zlim3d(0, 20)

    def __call__(self, event):
        if type(event) == matplotlib.backend_bases.PickEvent:
            ind = event.ind[0]
            self.selected_lines[:] = 0
            self.selected_lines[ind] = 1
            self.lines.set_color(self.normal_selected_color[self.selected_lines])
        elif type(event) == matplotlib.backend_bases.MouseEvent and event.button == 3:
            self.selected_lines[:] = 0
            self.lines.set_color(self.normal_selected_color[self.selected_lines])
        fig = event.canvas
        fig.draw_idle()

    def add_segment(self, segment):
        self.segments.append(segment)

    def sampleTrajectory(self):

        samples_irc = []
        samples_xyz = []

        self.selected_lines = np.zeros((len(self.segments)), dtype=int)
        for seg in self.segments:
            i = 0.0
            segment = []
            while i < seg.length:
                # samples_irc.append(self.c.move_to_pos(seg.val(n/seg.length), relative=False, move=False))
                p = seg.val(i/seg.length)
                segment.append((p[0], p[1], p[2]))
                i += 1.0
            p = seg.val(1.0)
            segment.append((p[0], p[1], p[2]))
            samples_xyz.append(segment)
        samples_xyz = Line3DCollection(samples_xyz, pickradius=5)
        samples_xyz.set_picker(True)
        self.lines = samples_xyz
        return samples_irc, samples_xyz



# if __name__ == '__main__':
#     e  = Editor()
#     s1 = LinSeg(np.zeros((3,)), np.array([2,3,0]))
#     s2 = LinSeg(np.array([2, 3, 0]), np.array([3,5,9]))
#     s3 = SplineSeg(np.array([3,5,9]), [-10.0, 2.0, 2.0], [1.0, -10.0, 5.0], [2.0, -5.0, 0.0], order=3)
#     s4 = SplineSeg(np.array([-3.0, 1.0, 6.0]), [-10.0, 2.0], [1.0, 5.0], [-2.0, 0.0], order=2)
#     s5 = CircSeg(np.zeros((3,)), np.ones((3,)), 10.0, -np.sqrt(25/2)*np.array([1.0,1.0,-1.0]), 360.0/180.0*np.pi)
#     s6 = CircSeg(np.zeros((3,)), np.ones((3,)), 5.0, -np.sqrt(25 / 2) * np.array([1.0, 1.0, -1.0]),
#                  360.0 / 180.0 * np.pi)
#     samples_irc, samples_xyz = e.sampleTrajectory([s1, s2, s3, s4, s5, s6])
#
#     e.plot_trajectory(samples_xyz)