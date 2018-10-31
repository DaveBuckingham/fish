import os
import sys



class Global_data():

    # CONSTANTS

    APPLICATION_NAME = 'IMU-Capture'
    VERSION = '0.3.3'
    APPLICATION_FULL_NAME = APPLICATION_NAME + ' ' + VERSION

    SAMPLE_FREQ_HZ = 200                             # EXPECTED SAMPLES PER SECOND
    SECONDS_PER_SAMPLE = 1 / SAMPLE_FREQ_HZ          # 0.005
    MS_PER_SAMPLE = int(SECONDS_PER_SAMPLE * 1000)   # 5, MUST BE AN INTEGER

    DATA_BUFFER_MIN = 1                  # >0
    DATA_BUFFER_MAX = 20000

    TRIGGER_DELAY_MIN = 0                # >=0
    TRIGGER_DELAY_MAX = 12000


    # OPTIONAL FEATURES

    USE_ENCODER = False
    SMART_FILE_LOADING = False


    # VARIABLES

    data_window_list = []
    calibration = None





