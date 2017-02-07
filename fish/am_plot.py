
import sys
from PyQt4 import QtGui, QtCore
from collections import deque

import pyqtgraph as pg


class Am_plot(pg.PlotWidget):

    def __init__(self, plot_data, parent=None):

        super(Am_plot, self).__init__()

        #self.setMinimumSize(3000, 100)
        #self.setMinimumHeight(500)
        #self.data = [ deque([]), deque([]), deque([]) ]
        self.plot_data = plot_data

        self.parent = parent;

        #self.x_scale_max = 300

        #font = QtGui.QFont('Serif', 7, QtGui.QFont.Light)
        #qp.setFont(font)

        self.colors = [QtGui.QColor(220, 0, 0), \
                      QtGui.QColor(0, 200, 0), \
                      QtGui.QColor(30, 30, 255)]


        #self.setDownsampling(ds=2, auto=False, mode='peak')

        left_axis   = self.getAxis('left')
        right_axis  = self.getAxis('right')
        top_axis    = self.getAxis('top')
        bottom_axis = self.getAxis('bottom')

        self.setClipToView(False)


        self.showGrid(True, True, 0.5)

        self.setMenuEnabled(enableMenu=False)

        # self.hideAxis('bottom')
        self.setMouseEnabled(x=False, y=False)

        self.enableAutoRange(True)

        #self.setXRange(0, 300, padding=0.00)


        self.hideButtons()

        #self.getViewBox().setMouseMode(self.getViewBox().RectMode)
        self.getViewBox().invertX(b=True)
        
        #self.setLimits(maxYRange=10000)


        self.curve_red   = self.plot([], [], pen=(255, 0, 0, 155))
        self.curve_green = self.plot([], [], pen=(0, 255, 0, 155))
        self.curve_blue  = self.plot([], [], pen=(0, 0, 255, 155))


    def clear_slot(self):
        pass

    def plot_slot(self):

        #print("DATA: " + str(self.plot_data[-1][0]) + "\t" + str(self.plot_data[-1][1]) + "\t" + str(self.plot_data[-1][2]))
        #x = range(len(self.data0), 0, -1)
        #print(self.plot_data)
        #x = range(len(self.plot_data[:][0]), 0, -1)
        self.curve_red.setData(self.plot_data[0])
        self.curve_green.setData(self.plot_data[1])
        self.curve_blue.setData(self.plot_data[2])
        #self.autoRange(padding=0.2)
        #self.setXRange(0, 300, padding=0.0)

