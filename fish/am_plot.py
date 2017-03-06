
import time
import random
import sys
from PyQt5 import QtGui, QtCore
from collections import deque

import pyqtgraph as pg


class Am_plot(pg.PlotWidget):

    def __init__(self, plot_data, lock, parent=None):

        super(Am_plot, self).__init__()

        self.plot_data = plot_data
        self.lock = lock

        self.parent = parent

        self.first = True

        #self.x_scale_max = 300

        #font = QtGui.QFont('Serif', 7, QtGui.QFont.Light)
        #qp.setFont(font)

        self.colors = [QtGui.QColor(220, 0, 0), \
                      QtGui.QColor(0, 200, 0), \
                      QtGui.QColor(30, 30, 255)]

        self.MAX_SAMPLES = 1000   # approx.


        left_axis   = self.getAxis('left')
        right_axis  = self.getAxis('right')
        top_axis    = self.getAxis('top')
        bottom_axis = self.getAxis('bottom')

        self.setClipToView(False)
        self.showGrid(True, True, 0.5)
        self.setMenuEnabled(enableMenu=True)
        #self.enableAutoRange(True)

        #self.setMinimumSize(3000, 100)
        # self.hideAxis('bottom')
        #self.setMinimumHeight(500)
        #self.data = [ deque([]), deque([]), deque([]) ]
        #self.setMouseEnabled(x=False, y=False)
        #self.setXRange(0, 300, padding=0.00)
        #self.hideButtons()
        #self.getViewBox().setMouseMode(self.getViewBox().RectMode)
        #self.getViewBox().invertX(b=True)
        #self.setLimits(maxYRange=10000)



        self.curve_red   = self.plot([], [], pen=(255, 0, 0, 155))
        self.curve_green = self.plot([], [], pen=(0, 255, 0, 155))
        self.curve_blue  = self.plot([], [], pen=(0, 0, 255, 155))


        #self.setDownsampling(ds=10)
        #self.setDownsampling(auto=True, mode='subsample')

    def downsample(self, data, num):
        indices = range(0, len(data))
        if (num < len(data)):
            ds = (data[::len(data)/num], indices[::len(data)/num])
            print(len(ds[0]))
            return ds
        return (data, indices)


    def plot_slot(self):

        #print("DATA: " + str(self.plot_data[-1][0]) + "\t" + str(self.plot_data[-1][1]) + "\t" + str(self.plot_data[-1][2]))
        #x = range(len(self.data0), 0, -1mean
        #print(self.plot_data)
        #x = range(len(self.plot_data[:][0]), 0, -1)

        if (not self.lock[0]):
            #self.curve_red.setData(self.plot_data[0])
            #self.curve_green.setData(self.plot_data[1])
            #self.curve_blue.setData(self.plot_data[2])
            d = self.downsample(self.plot_data[0], self.MAX_SAMPLES)
            self.curve_red.setData(d[1], d[0])
            d = self.downsample(self.plot_data[1], self.MAX_SAMPLES)
            self.curve_green.setData(d[1], d[0])
            d = self.downsample(self.plot_data[2], self.MAX_SAMPLES)
            self.curve_blue.setData(d[1], d[0])
            # self.curve_red.setData(self.downsample(self.plot_data[0], self.MAX_SAMPLES))
            # self.curve_green.setData(self.downsample(self.plot_data[1], self.MAX_SAMPLES))
            # self.curve_blue.setData(self.downsample(self.plot_data[2], self.MAX_SAMPLES))
        else:
            print("LOCKED")


        #self.autoRange(padding=0.2)
        #self.setXRange(0, 300, padding=0.0)

