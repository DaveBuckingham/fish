import pyqtgraph as pg
import logging

import PyQt5.QtWidgets

from imucapture.global_data import *


class Plot(pg.PlotWidget):

    def __init__(self, data, imu, modality, legend, parent=None):

        super().__init__(parent)

        self.data = data
        self.imu = imu
        self.modality = modality


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


        self.x_range = [x * Global_data.SECONDS_PER_SAMPLE for x in range(0, Global_data.DATA_BUFFER_MAX)]



    def plot_slot(self):

        self.curve_x_red.setData(  self.data.imu_data[self.imu, self.modality, 0, :])
        self.curve_y_green.setData(self.data.imu_data[self.imu, self.modality, 1, :])
        self.curve_z_blue.setData( self.data.imu_data[self.imu, self.modality, 2, :])



