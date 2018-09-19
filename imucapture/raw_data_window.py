#!/usr/bin/env python


import signal

import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

from imucapture.global_data import *

from imucapture.data_window      import Data_window
from imucapture.calib_dialog     import Calib_dialog
from imucapture.calib            import Calib
from imucapture.transform_dialog import Transform_dialog

class Raw_data_window(Data_window):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    recording = False

    finished_signal = PyQt5.QtCore.pyqtSignal()

    def __init__(self, data, title='missing title', parent = None):
        super().__init__(data, title, parent)

        self.data = data

        self.buttons['calibrate'] = PyQt5.QtWidgets.QPushButton('Calibrate')
        self.buttons['calibrate'].setMaximumWidth(Data_window.BUTTON_WIDTH)
        self.buttons['calibrate'].setToolTip('Use this data to calibrate the system')
        self.buttons['calibrate'].clicked.connect(self.calibrate_button_slot)
        self.button_layout.addWidget(self.buttons['calibrate'])
        self.buttons['calibrate'].setEnabled(False)

        self.buttons['transform'] = PyQt5.QtWidgets.QPushButton('Transform')
        self.buttons['transform'].setMaximumWidth(Data_window.BUTTON_WIDTH)
        self.buttons['transform'].setToolTip('Process the data by applying a filtering algorithm')
        self.buttons['transform'].clicked.connect(self.transform_button_slot)
        self.button_layout.addWidget(self.buttons['transform'])
        self.buttons['transform'].setEnabled(False)

    def activate_buttons(self):
        if (not self.recording):
            self.buttons['calibrate'].setEnabled(True)
            if(Global_data.calibration is not None and Global_data.calibration.imu_bases is not None):
                self.buttons['transform'].setEnabled(True)
            super().activate_buttons()


    def calibrate_button_slot(self):
        calib = Calib()
        if(calib.parse_data(self.data)):
            self.calib_dialog = Calib_dialog(calib)
            self.calib_dialog.show()

    # FILTER DATA BUTTON WAS PRESSED
    def transform_button_slot(self):
        self.transform_dialog = Transform_dialog(self.data)
        self.transform_dialog.finished_signal.connect(self.update)
        self.transform_dialog.show()


    # IF THE DATA WINDOW IS CLOSED WHILE STILL RECORDING, LET THE GUI KNOW TO STOP THE RECORDING
    def closeEvent(self, event):
        self.finished_signal.emit()
        super().closeEvent(event)


