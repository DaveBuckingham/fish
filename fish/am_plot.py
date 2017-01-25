
import sys
from PyQt4 import QtGui, QtCore
from collections import deque

import pyqtgraph as pg


class Am_plot(pg.PlotWidget):

    def __init__(self, data0, data1, data2, parent=None):


        super(Am_plot, self).__init__()


        #self.setMinimumSize(3000, 100)
        #self.setMinimumHeight(500)
        #self.data = [ deque([]), deque([]), deque([]) ]
        self.data0 = data0
        self.data1 = data1
        self.data2 = data2

        self.x_scale_max = 300

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

        self.setXRange(0, 300, padding=0.00)


        self.hideButtons()

        #self.getViewBox().setMouseMode(self.getViewBox().RectMode)
        self.getViewBox().invertX(b=True)
        
        #self.setLimits(maxYRange=10000)


        self.line1 = self.plot([], [], pen=(255, 0, 0, 155))
        self.line2 = self.plot([], [], pen=(0, 255, 0, 155))
        self.line3 = self.plot([], [], pen=(0, 0, 255, 155))


    def clear_slot(self):
        self.data = [ deque([]), deque([]), deque([]) ]
        #self.timestamps = deque([])

    def plot_slot(self):
        x = range(len(self.data[0]), 0, -1)
        self.line1.setData(x, self.data0)
        self.line2.setData(x, self.data1)
        self.line3.setData(x, self.data2)

    # RECEIVE A NEW DATA SAMPLE AND REPAINT GRAPH
    def data_slot(self, timestamp, values, refresh=True):

        #self.data[0].append(values[0])
        #self.data[1].append(values[1])
        #self.data[2].append(values[2])
        #self.timestamps.append(timestamp)

        # if (len(self.timestamps) > self.x_scale_max):
        #     self.data[0].popleft()
        #     self.data[1].popleft()
        #     self.data[2].popleft()
        #     self.timestamps.popleft()


        if (refresh):
            x = range(len(self.data[0]), 0, -1)
            #self.line1.setData(x, self.data[0])
            #self.line2.setData(x, self.data[1])
            #self.line3.setData(x, self.data[2])
            # self.autoRange(padding=0.2)
            # self.setXRange(0, 300, padding=0.0)

