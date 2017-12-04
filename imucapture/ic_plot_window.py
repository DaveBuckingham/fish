#!/usr/bin/env python

import sys
import os
import datetime
import signal
import time

import PyQt5.QtCore
import PyQt5.QtGui

from imucapture.ic_data import Ic_data
from imucapture.ic_plot import Ic_plot
from imucapture.ic_global import *

class Ic_plot_window(PyQt5.QtGui.QWidget):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, data, parent = None):
        super(Ic_plot_window, self).__init__(parent)


        # SET WINDOW TITLE
        self.setWindowTitle(data.data_description_string)

        self.data = data


        ########################
        #       VARIABLES      #
        ########################


        # CURRENTLY RECORDING DATA?
        self.recording = False

        self.plots_layout = PyQt5.QtGui.QGridLayout()
        self.setLayout(self.plots_layout)

        self.plots = []

        self.make_plots()



    def make_plots(self):

        #self.clear_layout(self.plots_layout)

        # READ LABELS FROM DATA...

        label = PyQt5.QtGui.QLabel(self.data.accel_units_string)
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 1)

        label = PyQt5.QtGui.QLabel(self.data.gyro_units_string)
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 2)

        label = PyQt5.QtGui.QLabel(self.data.mag_units_string)
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 3)

        self.plots = []
        for i in (range(0, self.data.num_imus)):
            plot_a = Ic_plot(self.data.imu_data['imus'][i]['accel'], self.data.data_lock, True, self)
            plot_g = Ic_plot(self.data.imu_data['imus'][i]['gyro'], self.data.data_lock, False, self)
            plot_m = Ic_plot(self.data.imu_data['imus'][i]['mag'], self.data.data_lock, False, self)
            self.plots.append(plot_a)
            self.plots.append(plot_g)
            self.plots.append(plot_m)
            self.plots_layout.addWidget(plot_a, i+1, 1)
            self.plots_layout.addWidget(plot_g, i+1, 2)
            self.plots_layout.addWidget(plot_m, i+1, 3)

            label = PyQt5.QtGui.QLabel("IMU " + str(i+1))
            label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
            self.plots_layout.addWidget(label, i+1, 0)




    # UPDATE PLOTS
    def update(self):
        for p in self.plots:
            p.plot_slot()


    # SYNC NUMBER OF IMUS WITH RECEIVER AND CREATE THE CORRECT NUMBER OF PLOTS
    def numimus_slot(self, num_imus):
        self.num_imus = num_imus
        self.make_plots()


