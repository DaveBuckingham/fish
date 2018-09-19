#!/usr/bin/env python


import logging

import PyQt5.QtCore
import PyQt5.QtWidgets

from imucapture.data import Data
from imucapture.plot import Plot
from imucapture.global_data import *

class Data_window(PyQt5.QtWidgets.QWidget):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 100

    BUTTON_WIDTH = 200

    def __init__(self, data, title, parent = None):
        super().__init__(parent)

        Global_data.data_window_list.append(self)

        # SET WINDOW TITLE
        self.setWindowTitle(title)

        self.data = data

        self.buttons = {}



        ########################
        #       VARIABLES      #
        ########################


        self.plots_layout = PyQt5.QtWidgets.QGridLayout()

        self.button_layout = PyQt5.QtWidgets.QHBoxLayout()

        top_layout = PyQt5.QtWidgets.QVBoxLayout()
        top_layout.addStretch()
        top_layout.addLayout(self.plots_layout)
        top_layout.addLayout(self.button_layout)


        self.setLayout(top_layout)

        self.buttons['save'] = PyQt5.QtWidgets.QPushButton('Save')
        self.buttons['save'].setMaximumWidth(Data_window.BUTTON_WIDTH)
        self.buttons['save'].setToolTip('Save the current data to hdf5 file')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        self.button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['close'] = PyQt5.QtWidgets.QPushButton('Close')
        self.buttons['close'].setMaximumWidth(Data_window.BUTTON_WIDTH)
        self.buttons['close'].setToolTip('Exit the program')
        self.buttons['close'].clicked.connect(self.close_button_slot)
        self.button_layout.addWidget(self.buttons['close'])
        self.buttons['close'].setEnabled(False)





        self.plots = []

        self.make_plots()



    def make_plots(self):

        #self.clear_layout(self.plots_layout)

        # READ LABELS FROM DATA...

        label = PyQt5.QtWidgets.QLabel(self.data.accel_units_string)
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 1)

        label = PyQt5.QtWidgets.QLabel(self.data.gyro_units_string)
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 2)

        label = PyQt5.QtWidgets.QLabel(self.data.mag_units_string)
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 3)

        self.plots = []
        for i in (range(0, self.data.num_imus)):
            plot_a = Plot(self.data, i, Data.ACCEL_INDEX, True)
            plot_g = Plot(self.data, i, Data.GYRO_INDEX, False)
            plot_m = Plot(self.data, i, Data.MAG_INDEX, False)
            self.plots.append(plot_a)
            self.plots.append(plot_g)
            self.plots.append(plot_m)
            self.plots_layout.addWidget(plot_a, i+1, 1)
            self.plots_layout.addWidget(plot_g, i+1, 2)
            self.plots_layout.addWidget(plot_m, i+1, 3)

            label = PyQt5.QtWidgets.QLabel("IMU " + str(i+1))
            label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
            self.plots_layout.addWidget(label, i+1, 0)




    # UPDATE PLOTS
    def update(self):
        if (self.data.mutex.tryLock()):
            for p in self.plots:
                p.plot_slot()
            self.data.mutex.unlock()
        else:
            logging.warning("Missed plot update: data being written");


    # SYNC NUMBER OF IMUS WITH RECEIVER AND CREATE THE CORRECT NUMBER OF PLOTS
    def numimus_slot(self, num_imus):
        self.num_imus = num_imus
        self.make_plots()


    def close_button_slot(self):
        logging.info("closing data window")
        #self.deleteLater()
        self.close()

    def activate_buttons(self):
        self.buttons['save'].setEnabled(True)
        self.buttons['close'].setEnabled(True)


    def closeEvent(self, event):
        if self in Global_data.data_window_list:
            # DELETE THE REFERENCE TO THE WINDOW
            Global_data.data_window_list.remove(self)
        else:
            logging.error("closing data window. should be in global_data list but isn't!")
        event.accept()

    # GET A FILENAME AND TYPE FROM THE USER AND THEN SAVE THE DATA BUFFER
    def save_button_slot(self):

        options = PyQt5.QtWidgets.QFileDialog.Options() | PyQt5.QtWidgets.QFileDialog.DontUseNativeDialog
        filename, filetype = PyQt5.QtWidgets.QFileDialog.getSaveFileName(parent=self,
                                                                     caption="Save data",
                                                                     filter="*.hdf5",
                                                                     options=options)

        if filename:
            filename = str(filename)
            Global_data.last_data_path = os.path.dirname(filename)

            (base, extension) = os.path.splitext(filename)
            if (not extension):
                filename = base + '.hdf5'

            self.data.save_file(filename)

            logging.info("saved " + filename)
            self.setWindowTitle(filename)
            return True


