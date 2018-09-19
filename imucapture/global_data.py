import os
import sys



class Global_data():

    # CONSTANTS

    VERSION = '0.2.1'

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




    calibration = None
