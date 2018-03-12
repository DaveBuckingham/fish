#!/usr/bin/env python


from imucapture.ic_data_window import Ic_data_window
from imucapture.ic_global import *

class Ic_transformed_data_window(Ic_data_window):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, data, title='unsaved transformed data', parent = None):
        super().__init__(data, title, parent)



