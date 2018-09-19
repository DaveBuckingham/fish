import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

from imucapture.get_basis import Get_basis
from imucapture.global_data import *
from imucapture.data import Data

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Calib():

    def __init__(self):


        self.get_basis = Get_basis()

        self.imu_bases = None

        self.initial_gravity = None
        self.still_accel = None
        self.still_gyro = None

        self.intervals = None




    def parse_data(self, calib_data):



        self.intervals = self.get_basis.get_intervals(calib_data)
        imu_bases = self.get_basis.get_bases(calib_data, self.intervals)


        if ((imu_bases is not None) and (len(imu_bases) == calib_data.num_imus)):
            logging.info("extracted basis vectors:")
            for imu in range(0, calib_data.num_imus):
                logging.info("imu " + str(imu) + ":\n" + str(imu_bases[imu][0]) + "\n" + str(imu_bases[imu][1]) + "\n" + str(imu_bases[imu][2]))
            self.imu_bases = imu_bases
            
            still_start = self.intervals[0][0]
            still_end = self.intervals[0][1]

            self.initial_gravity = numpy.mean(calib_data.imu_data[0, Data.ACCEL_INDEX, :, still_start:still_end], axis=1)
            self.still_accel = calib_data.imu_data[0, Data.ACCEL_INDEX, :, still_start:still_end].transpose()
            self.still_gyro = calib_data.imu_data[0, Data.GYRO_INDEX, :, still_start:still_end].transpose()

            return True

        else:
            logging.error("failed to extract bases for every imu")
            return False



