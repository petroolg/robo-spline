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

import wx
import numpy
import matplotlib
from wx.lib.masked import NumCtrl


from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wxagg import NavigationToolbar2Wx as NavigationToolbar

from trajectory_editor import *

class CoordInputPanel(wx.Panel):

    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        self.box = wx.BoxSizer(wx.HORIZONTAL)
        labels_txt = ['x','y','z']
        self.inputs = []
        for l in labels_txt:
            lab = wx.StaticText(self, -1, l)
            self.inputs.append(NumCtrl(self, -1, integerWidth=3))
            self.box.Add(lab, 1)
            self.box.Add(self.inputs[-1], 1)
            self.inputs[-1].Bind(wx.EVT_LEFT_UP, self.myonClick)
        self.SetSizer(self.box)
        self.Layout()

    def get_input(self):
        return [self.inputs[0].GetValue(), self.inputs[1].GetValue(), self.inputs[2].GetValue()]

    def set_input(self, value):
        self.inputs[0].SetValue(value[0])
        self.inputs[1].SetValue(value[1])
        self.inputs[2].SetValue(value[2])

    def myonClick(self, event):
        self.Parent.onClick(self)
        event.Skip()

class TrajectoryPanel(wx.Panel):

    def __init__(self, parent, style=None):
        wx.Panel.__init__(self, parent, style=style)
        self.segments = []
        self.box = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.box)

    def add_segment(self, segment):
        seg = LinSegButton(self, segment)
        self.segments.append(seg)
        self.box.Add(seg, 0, wx.ALIGN_LEFT)
        self.Parent.Parent.matplot_panel.e.add_segment(segment)

    def onClick(self, i_seg, code=None):
        for s in self.segments:
            s.BackgroundColour = wx.NullColour
        if i_seg:
            i_seg.BackgroundColour = wx.LIGHT_GREY
        self.Refresh()

class LinSegButton(wx.Panel):

    def __init__(self, parent, segment):
        wx.Panel.__init__(self, parent)
        self.box = wx.BoxSizer(wx.VERTICAL)
        label_start = wx.StaticText(self, -1, 'Starting point:')
        label_end = wx.StaticText(self, -1, 'Ending point:')
        self.input_start = CoordInputPanel(self)
        self.input_end = CoordInputPanel(self)
        if segment:
            self.input_start.set_input(segment.start)
            self.input_end.set_input(segment.end)
        self.box.Add(label_start, 0, wx.ALIGN_LEFT)
        self.box.Add(self.input_start,0, wx.ALIGN_LEFT)
        self.box.Add(label_end, 0, wx.ALIGN_LEFT)
        self.box.Add(self.input_end, 0, wx.ALIGN_LEFT)
        self.SetSizer(self.box)
        self.Bind(wx.EVT_LEFT_UP, self.onClick)
        self.Layout()

    def onClick(self, event):
        self.Parent.onClick(self)

class TestFrame(wx.Frame):
    def __init__(self,parent,title):
        wx.Frame.__init__(self,parent,title=title,size=(800,600))
        self.splitter = wx.SplitterWindow(self)
        self.traj_panel = TrajectoryPanel(self.splitter, style=wx.SUNKEN_BORDER)
        self.matplot_panel = MatplotPanel(self.splitter)
        self.splitter.SplitVertically(self.traj_panel,self.matplot_panel,150)
        # self.statusbar = self.CreateStatusBar()

        self.segments = []

class MatplotPanel(wx.Panel):

    def __init__(self, parent):
        wx.Panel.__init__(self, parent,-1,size=(50,50))
        self.parent = parent
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.sizer)

        self.e = Editor()
        self.canvas = FigureCanvas(self, -1, self.e.figure)
        self.toolbar = NavigationToolbar(self.canvas)

        self.e.figure.canvas.mpl_connect('button_press_event', self.e)
        self.e.figure.canvas.mpl_connect('pick_event', self.e)
        self.e.axes.mouse_init()

        # self.button = wx.Button(self, -1, "Change plot")
        # self.button.Bind(wx.EVT_BUTTON, self.changePlot)


        # self.sizer.Add(self.button, 0, wx.EXPAND)
        self.sizer.Add(self.canvas, 1, wx.LEFT | wx.TOP | wx.GROW)
        self.sizer.Add(self.toolbar, 0, wx.EXPAND | wx.BOTTOM)

        self.toolbar.update()

if __name__ == '__main__':

    app = wx.App(redirect=False)
    frame = TestFrame(None, 'Trajectory editor v')
    frame.traj_panel.add_segment(LinSeg(np.zeros((3,)), np.array([2, 3, 0])))
    frame.traj_panel.add_segment(LinSeg(np.array([2, 3, 0]), np.array([3, 5, 9])))
    frame.traj_panel.add_segment(
        SplineSeg(np.array([3, 5, 9]), [-10.0, 2.0, 2.0], [1.0, -10.0, 5.0], [2.0, -5.0, 0.0], order=3))
    frame.traj_panel.add_segment(SplineSeg(np.array([-3.0, 1.0, 6.0]), [-10.0, 2.0], [1.0, 5.0], [-2.0, 0.0], order=2))
    frame.traj_panel.add_segment(CircSeg(np.zeros((3,)), np.ones((3,)), 10.0, -np.sqrt(25 / 2) * np.array([1.0, 1.0, -1.0]),
                                 360.0 / 180.0 * np.pi))
    frame.traj_panel.add_segment(CircSeg(np.zeros((3,)), np.ones((3,)), 5.0, -np.sqrt(25 / 2) * np.array([1.0, 1.0, -1.0]),
                                 360.0 / 180.0 * np.pi))

    samples_irc, samples_xyz = frame.matplot_panel.e.sampleTrajectory()
    frame.matplot_panel.e.axes.add_collection3d(samples_xyz)

    frame.Show()
    app.MainLoop()
