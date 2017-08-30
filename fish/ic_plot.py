import pyqtgraph as pg

import PyQt5.QtGui


class Ic_plot(pg.PlotWidget):

    def __init__(self, plot_data, lock, parent=None):

        super(Ic_plot, self).__init__()

        self.plot_data = plot_data
        self.lock = lock

        self.parent = parent

        self.first = True


        self.colors = [PyQt5.QtGui.QColor(220, 0, 0), \
                       PyQt5.QtGui.QColor(0, 200, 0), \
                       PyQt5.QtGui.QColor(30, 30, 255)]

        self.MAX_SAMPLES = 1000   # approx.

        self.setMinimumWidth(250)


        self.setClipToView(False)
        self.showGrid(True, True, 0.5)
        self.setMenuEnabled(enableMenu=True)


        self.addLegend()

        self.curve_x_red   = self.plot([], [], pen=(255, 0, 0, 165), name='x')
        self.curve_y_green = self.plot([], [], pen=(0, 255, 0, 155), name='y')
        self.curve_z_blue  = self.plot([], [], pen=(30, 60, 255, 210), name='z')


    def downsample(self, data, num):
        indices = range(0, len(data))
        if (num < len(data)):
            ds = (data[::len(data)/num], indices[::len(data)/num])
            print(len(ds[0]))
            return ds
        return (data, indices)


    def plot_slot(self):

        if (not self.lock[0]):
            self.curve_x_red.setData(self.plot_data[0])
            self.curve_y_green.setData(self.plot_data[1])
            self.curve_z_blue.setData(self.plot_data[2])

        else:
            print("LOCKED")


