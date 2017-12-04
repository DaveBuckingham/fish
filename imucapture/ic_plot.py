import pyqtgraph as pg

import PyQt5.QtGui

from imucapture.ic_global import *


class Ic_plot(pg.PlotWidget):

    def __init__(self, plot_data, lock, legend, parent=None):

        super(Ic_plot, self).__init__()

        self.plot_data = plot_data
        self.lock = lock

        self.parent = parent

        self.first = True


        self.colors = [PyQt5.QtGui.QColor(220, 0, 0), \
                       PyQt5.QtGui.QColor(0, 200, 0), \
                       PyQt5.QtGui.QColor(30, 30, 255)]

        self.setMinimumWidth(250)


        self.setClipToView(False)
        self.showGrid(True, True, 0.5)
        self.setMenuEnabled(enableMenu=True)


        if(legend):
            self.addLegend()

        # DOWNSAMPLING SEEMS TO DO SOMETHING LIKE:
        # dx = float(x[-1]-x[0]) / (len(x)-1)
        # THUS TO AVOID OUT-OF-BOUNDS OR DIVIDE-BY-ZERO ERRORS, WE PUT A COUPLE ELEMENTS TO START
        self.curve_x_red   = self.plot([0,0], pen=(255, 0, 0, 165),   name='x', downsampleMethod='peak', autoDownsample=True)
        self.curve_y_green = self.plot([0,0], pen=(0, 255, 0, 155),   name='y', downsampleMethod='peak', autoDownsample=True)
        self.curve_z_blue  = self.plot([0,0], pen=(30, 60, 255, 210), name='z', downsampleMethod='peak', autoDownsample=True)


        # WITHOUT DOWNSAMPLING?
        # self.curve_x_red   = self.plot([], [], pen=(255, 0, 0, 165), name='x')
        # self.curve_y_green = self.plot([], [], pen=(0, 255, 0, 155), name='y')
        # self.curve_z_blue  = self.plot([], [], pen=(30, 60, 255, 210), name='z')

        self.x_range = [x * 0.005 for x in range(0, Ic_global.DATA_BUFFER_MAX)]



    def downsample(self, data, num):
        indices = range(0, len(data))
        if (num < len(data)):
            ds = (data[::len(data)/num], indices[::len(data)/num])
            print(len(ds[0]))
            return ds
        return (data, indices)


    def plot_slot(self):

        if (not self.lock[0]):
            self.curve_x_red.setData(self.x_range[:len(self.plot_data[0])], self.plot_data[0])
            self.curve_y_green.setData(self.x_range[:len(self.plot_data[1])], self.plot_data[1])
            self.curve_z_blue.setData(self.x_range[:len(self.plot_data[2])], self.plot_data[2])

        else:
            print("LOCKED")


