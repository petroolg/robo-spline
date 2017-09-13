import numpy as np
from CRS_commander import Commander
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Segment:

    def val(self, t): raise NotImplementedError
    def length(self): raise NotImplementedError

class LinSeg(Segment):

    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.len = np.linalg.norm(end-start)
        self.vec = (end-start)

    def val(self, t):
        return self.start + t * self.vec

    @property
    def length(self):
        return self.len

class SplineSeg(Segment):
    def __init__(self, start, cx, cy, cz, order=2):
        assert order == 2 or order == 3, 'Constructor supports only 2nd and 3rd order polynomials'
        assert len(cx) == len(cy) == len(cz) == order, 'Number of parameters must be equal to order of polynomial' \
                                                       'cx: %d, cy: %d, cz: %d, order = %d'%(len(cx), len(cy), len(cz), order)
        self.start = start
        self.cx = cx
        self.cy = cy
        self.cz = cz
        self.ord = order

    @property
    def length(self):
        if hasattr(self, 'l'):
            return self.l

        l, t, delta = 0.0, 0.0, 0.1
        while t < 1.0:
            while True:
                step = self.val(t + delta) - self.val(t)
                step_len = step[0]*step[0] + step[1]*step[1] + step[2]*step[2]
                if step_len > 4.0: delta /= 2
                else: break
            t += delta
            l += step_len
        self.l = l
        return l

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
        self.l = angle * 2 * np.pi * r

        self.axis = axis / np.linalg.norm(axis)
        self.a = center - start
        self.a /= np.linalg.norm(self.a)
        self.b = np.cross(self.a, self.axis)
        self.b /= np.linalg.norm(self.b)

    def val(self, t):
        return self.c + self.r * np.cos(self.angle * t) * self.a + self.r * np.sin(self.angle * t) * self.b

    @property
    def length(self):
        return self.l

class Editor:

    def __init__(self):
        # self.c = commander
        pass

    def sampleTrajectory(self, segments):

        samples_irc = []
        samples_xyz = []
        for seg in segments:
            i = 0.0
            while i < seg.length:
                # samples_irc.append(self.c.move_to_pos(seg.val(n/seg.length), relative=False, move=False))
                samples_xyz.append(seg.val(i/seg.length))
                i += 1.0
            samples_xyz.append(seg.val(1.0))

        return samples_irc, samples_xyz

    def plot_trajectory(self, trajectory):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        trajectory = np.array(trajectory)
        ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2])
        plt.show()

if __name__ == '__main__':
    e  = Editor()
    s1 = LinSeg(np.zeros((3,)), np.array([2,3,0]))
    s2 = LinSeg(np.array([2, 3, 0]), np.array([3,5,9]))
    s3 = SplineSeg(np.array([3,5,9]), [-10.0, 2.0, 2.0], [1.0, -10.0, 5.0], [2.0, -5.0, 0.0], order=3)
    s4 = SplineSeg(np.array([-3.0, 1.0, 6.0]), [-10.0, 2.0], [1.0, 5.0], [-2.0, 0.0], order=2)
    s5 = CircSeg(np.zeros((3,)), np.ones((3,)), 10.0, -np.sqrt(25/2)*np.array([1.0,1.0,-1.0]), 360.0/180.0*np.pi)
    s6 = CircSeg(np.zeros((3,)), np.ones((3,)), 5.0, -np.sqrt(25 / 2) * np.array([1.0, 1.0, -1.0]),
                 360.0 / 180.0 * np.pi)
    samples_irc, samples_xyz = e.sampleTrajectory([s1, s2, s3, s4, s5, s6])

    e.plot_trajectory(samples_xyz)