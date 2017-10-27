import os
import sys


class Ic_global():

    # CONSTANTS

    USE_ENCODER = True

    APPLICATION_NAME = 'IMU-Capture'

    here = os.path.abspath(os.path.dirname(__file__))

    with open(os.path.join(here, 'VERSION')) as version_file:
        VERSION = version_file.read().strip()


    APPLICATION_FULL_NAME = APPLICATION_NAME + ' ' + VERSION




    # VARIABLES

    last_file_path = ''


