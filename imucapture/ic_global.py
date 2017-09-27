import os
import sys


class Ic_global():

    # CONSTANTS

    USE_ENCODER = False

    APPLICATION_NAME = 'IMU-Capture'

    here = os.path.abspath(os.path.dirname(__file__))

    with open(os.path.join(here, 'VERSION')) as version_file:
        VERSION = version_file.read().strip()


    APPLICATION_FULL_NAME = APPLICATION_NAME + ' ' + VERSION


    DATA_BUFFER_MIN = 1                  # >0
    DATA_BUFFER_MAX = 10000




    # VARIABLES

    last_file_path = ''


