
import sys
from PyQt4 import QtGui, QtCore


class Communicate(QtCore.QObject):
    
    updateBW = QtCore.pyqtSignal(int)


class Am_plot(QtGui.QWidget):

    def __init__(self):      
        super(Am_plot, self).__init__()
        
        self.setMinimumSize(300, 100)
        self.data = []
        self.x_scale = 100
        self.y_scale = 10

        self.colors = [QtGui.QColor(220, 0, 0), \
                      QtGui.QColor(0, 200, 0), \
                      QtGui.QColor(30, 30, 255)]

        self.width = self.size().width()
        self.height = self.size().height()



    def data_slot(self, values):
        self.data = values
        self.repaint()


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
            for i in range(0, len(self.data[0])):
                path = QtGui.QPainterPath()
                path.moveTo(0, self.height/2)
                qp.setPen(self.colors[i % len(self.colors)])
                num_samples = min(len(self.data), self.x_scale + 1)
                x = 0
                for j in range(-num_samples, 0):
                    path.lineTo(x, (self.height/2) - (self.data[j][i] * self.y_scale))
                    x += float(self.width) / self.x_scale
                qp.drawPath(path)
                

        qp.end()
