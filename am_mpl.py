
import sys
from PyQt4 import QtGui, QtCore
from collections import deque


import random
from matplotlib.backends import qt_compat
use_pyside = qt_compat.QT_API == qt_compat.QT_API_PYSIDE
if use_pyside:
    from PySide import QtGui, QtCore
else:
    from PyQt4 import QtGui, QtCore

from numpy import arange, sin, pi
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class Am_mpl(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        self.axes.hold(True)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

        #self.setMinimumSize(300, 100)
        self.data = [ deque([]), deque([]), deque([]) ]
        self.timestamps = deque([])
        self.x_scale = 300
        self.y_scale = 10

        #font = QtGui.QFont('Serif', 7, QtGui.QFont.Light)
        #qp.setFont(font)

        self.colors = [QtGui.QColor(220, 0, 0), \
                      QtGui.QColor(0, 200, 0), \
                      QtGui.QColor(30, 30, 255)]


        #self.axes.set_xlim([0, 400])



    def clear_slot(self):
	# DEQUE ALLOWS FAST ADD OR REMOVE FROM EITHER END
        #self.data = deque([[],[],[]]) 
        self.data = [ deque([]), deque([]), deque([]) ]
        self.timestamps = deque([])
        self.axes.cla()
        self.draw()

    # RECEIVE A NEW DATA SAMPLE AND REPAINT GRAPH
    def data_slot(self, timestamp, values, refresh=True):
        #if (len(self.data) > 0):
        self.data[0].append(values[0])
        self.data[1].append(values[1])
        self.data[2].append(values[2])
        self.timestamps.append(timestamp)
        if (len(self.timestamps) > self.x_scale):
            self.timestamps.popleft()
            self.data[0].popleft()
            self.data[1].popleft()
            self.data[2].popleft()
        print len(self.data[0])
        if (refresh):
            self.axes.cla()
            self.axes.plot(self.timestamps, self.data[0], 'r', linewidth=1)
            self.axes.plot(self.timestamps, self.data[1], 'g', linewidth=1)
            self.axes.plot(self.timestamps, self.data[2], 'b', linewidth=1)
            self.draw()


