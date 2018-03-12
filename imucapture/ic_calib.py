import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

from imucapture.ic_data import Ic_data
from imucapture.ic_get_basis import Ic_get_basis
from imucapture.ic_global import *

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_calib():


    def __init__(self):


        self.get_basis = Ic_get_basis()

        self.imu_bases = None

        self.initial_gravity = None
        self.still_accel = None
        self.still_gyro = None

        self.intervals = None




    def parse_calibration(self):

        calib_data = Ic_data()

        options = PyQt5.QtWidgets.QFileDialog.Options() | PyQt5.QtWidgets.QFileDialog.DontUseNativeDialog
        filename, searchtype = PyQt5.QtWidgets.QFileDialog.getOpenFileName(parent=None, 
                                                                       caption="Choose a file",
                                                                       directory=Ic_global.last_file_path,
                                                                       filter="*.csv *.hdf5",
                                                                       options=options)

        if filename:
            filename = str(filename)
            Ic_global.last_data_path = os.path.dirname(filename)
            prefix, extension = os.path.splitext(filename)

            logging.info("loading " + filename)

            if (extension == ".hdf5"):
                if not calib_data.load_hdf5_file(filename):
                    logging.error("load failed\n")
                    return False
            elif (extension == ".csv"):
                if not calib_data.load_csv_file(filename):
                    logging.error("load failed\n")
                    return False
            else:
                logging.error("invalid file extension: " + extension + "\n")
                return False
        else:
            return False

        self.intervals = self.get_basis.get_intervals(calib_data)
        imu_bases = self.get_basis.get_bases(calib_data, self.intervals)


        if ((imu_bases is not None) and (len(imu_bases) == calib_data.num_imus)):
            logging.info("extracted basis vectors:")
            for imu in range(0, calib_data.num_imus):
                logging.info("imu " + str(imu) + ":\n" + str(imu_bases[imu][0]) + "\n" + str(imu_bases[imu][1]) + "\n" + str(imu_bases[imu][2]))
            self.imu_bases = imu_bases
            
            still_start = self.intervals[0][0]
            still_end = self.intervals[0][1]

            calib_accel = calib_data.as_list_of_triples(0, 'accel')
            calib_gyro = calib_data.as_list_of_triples(0, 'gyro')

            self.initial_gravity = numpy.mean(calib_accel[still_start:still_end], axis=0)
            self.still_accel = numpy.array(calib_accel[still_start:still_end])
            self.still_gyro = numpy.array(calib_gyro[still_start:still_end])
            return True

        else:
            logging.error("failed to extract bases for every imu")
            return False



