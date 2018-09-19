#!/usr/bin/env python


from imucapture.data_window import Data_window
from imucapture.global_data import *

class Transformed_data_window(Data_window):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, data, title='unsaved transformed data', parent = None):
        super().__init__(data, title, parent)



