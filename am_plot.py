
import sys
from PyQt4 import QtGui, QtCore
from collections import deque


#class Communicate(QtCore.QObject):
#    updateBW = QtCore.pyqtSignal(int)


class Am_plot(QtGui.QWidget):

    def __init__(self):      
        super(Am_plot, self).__init__()
        
        self.setMinimumSize(300, 100)
        self.data = deque([])
        self.x_scale = 300
        self.y_scale = 10

	# DRAW RED, GREEN, AND BLUE LINES
        self.colors = [QtGui.QColor(220, 0, 0), \
                      QtGui.QColor(0, 200, 0), \
                      QtGui.QColor(30, 30, 255)]

        self.width = self.size().width()
        self.height = self.size().height()

    # RESET GRAPH
    def clear_slot(self):
	# DEQUE ALLOWS FAST ADD OR REMOVE FROM EITHER END
        self.data = deque([]) 
        self.repaint()


    # RECEIVE A NEW DATA SAMPLE AND REPAINT GRAPH
    def data_slot(self, values, refresh=True):
        self.data.append(values)
        if (len(self.data) > self.x_scale):
            self.data.popleft()
        if (refresh):
            self.repaint()


    # REPAINT THE GRAPH
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)

        font = QtGui.QFont('Serif', 7, QtGui.QFont.Light)
        qp.setFont(font)

        self.width = self.size().width()
        self.height = self.size().height()

        pen = QtGui.QPen(QtGui.QColor(20, 20, 20), 1, QtCore.Qt.SolidLine)
        qp.setPen(pen)
        qp.setBrush(QtGui.QColor(120,120,120))
        qp.drawRect(0, 0, self.width-1, self.height-1)
        qp.setBrush(QtCore.Qt.NoBrush)

        if (len(self.data) > 0):
	    # FOR EACH OF THREE DIMENSIONS
            for i in range(0, len(self.data[0])):
                path = QtGui.QPainterPath()
                path.moveTo(0, self.height/2)
                qp.setPen(self.colors[i % len(self.colors)])
                num_samples = min(len(self.data), self.x_scale + 1)
                x = 0
		# FOR EACH DATA SAMPLE
                for j in range(-num_samples, 0):
                    path.lineTo(x, (self.height/2) - (self.data[j][i] * self.y_scale))
                    x += float(self.width) / self.x_scale
                qp.drawPath(path)
                

        qp.end()
