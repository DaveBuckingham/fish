import os
class Ic_global():

    # CONSTANTS

    USE_ENCODER = False

    APPLICATION_NAME = 'IMU-Capture'

    with open('../VERSION') as version_file:
        VERSION = version_file.read().strip()

    APPLICATION_FULL_NAME = APPLICATION_NAME + ' ' + VERSION




    # VARIABLES

    last_file_path = ''


