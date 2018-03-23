import os
import sys

import PyQt5.QtCore
import PyQt5.QtWidgets

from imucapture.ic_calib import Ic_calib


class Ic_global():

    # CONSTANTS

    VERSION = '0.1'

    USE_ENCODER = False

    SAMPLE_FREQ_HZ = 200                             # EXPECTED SAMPLES PER SECOND
    SECONDS_PER_SAMPLE = 1 / SAMPLE_FREQ_HZ          # 0.005
    MS_PER_SAMPLE = int(SECONDS_PER_SAMPLE * 1000)   # 5, MUST BE AN INTEGER


    APPLICATION_NAME = 'IMU-Capture'

    here = os.path.abspath(os.path.dirname(__file__))

    APPLICATION_FULL_NAME = APPLICATION_NAME + ' ' + VERSION

    DATA_BUFFER_MIN = 1                  # >0
    DATA_BUFFER_MAX = 20000



    # VARIABLES
    data_window_list = []



    # FUNCTIONS
    def enable_layout(layout, state):
        items = (layout.itemAt(i) for i in range(layout.count())) 
        for w in items:
            if isinstance(w, PyQt5.QtWidgets.QLayout):
                Ic_global.enable_layout(w, state)
            else:
                w.widget().setEnabled(state)

    calibration = Ic_calib()
