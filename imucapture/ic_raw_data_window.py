#!/usr/bin/env python


import signal

import PyQt5.QtCore
import PyQt5.QtWidgets

from imucapture.ic_data_window import Ic_data_window
from imucapture.ic_global import *
from imucapture.ic_transform_dialog import Ic_transform_dialog

class Ic_raw_data_window(Ic_data_window):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    recording = False

    finished_signal = PyQt5.QtCore.pyqtSignal()

    def __init__(self, data, title='missing title', parent = None):
        super().__init__(data, title, parent)

        self.data = data

        self.buttons['filter'] = PyQt5.QtWidgets.QPushButton('Filter')
        self.buttons['filter'].setMaximumWidth(Ic_data_window.BUTTON_WIDTH)
        self.buttons['filter'].setToolTip('Process the current data by applying a filtering algorithm')
        self.buttons['filter'].clicked.connect(self.filter_button_slot)
        self.button_layout.addWidget(self.buttons['filter'])
        self.buttons['filter'].setEnabled(False)


    # FILTER DATA BUTTON WAS PRESSED
    def filter_button_slot(self):
        transform_dialog = Ic_transform_dialog(self.data)
        transform_dialog.finished_signal.connect(self.update)
        transform_dialog.show()


    # IF THE DATA WINDOW IS CLOSED WHILE STILL RECORDING, LET THE GUI KNOW TO STOP THE RECORDING
    def closeEvent(self, event):
        self.finished_signal.emit()
        super().closeEvent(event)


