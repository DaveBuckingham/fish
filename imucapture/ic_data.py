import h5py
import csv
import logging

import sys

import numpy
import math

import time

import PyQt5.QtCore

from imucapture.ic_global import Ic_global


try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_data(PyQt5.QtCore.QObject):


    ACCEL_INDEX = 0
    GYRO_INDEX  = 1
    MAG_INDEX   = 2

    def __init__(self, dataset_type, data, num_samples):
        super(Ic_data, self).__init__()

        self.mutex = PyQt5.QtCore.QMutex()

        self.set_dataset_type(dataset_type)

        self.imu_data = data

        self.num_imus = self.imu_data.shape[0]
        self.num_samples = num_samples
        self.total_samples = self.num_samples


    @classmethod
    def from_file(cls, filename):

        start = time.time()

        with h5py.File(filename, 'r') as datafile:
            if datafile is None:
                logging.error("couldn't load file")
                return None

            if ('imu0' not in datafile['data']):
                logging.error("couldn't load file: invalid format")
                return None


            data = numpy.array([numpy.stack((datafile['data']['imu0']['accel'],
                                             datafile['data']['imu0']['gyro'],
                                             datafile['data']['imu0']['mag']
                                           ))
                              ])

            i = 1
            while ('imu' + str(i) in datafile['data']):
                imu_str = 'imu' + str(i)
                data = numpy.append(data,
                                    numpy.stack((datafile['data'][imu_str]['accel'],
                                                 datafile['data'][imu_str]['gyro'],
                                                 datafile['data'][imu_str]['mag']
                                               )),
                                    0)
                i += 1

            dataset_type = datafile['data'].attrs['description']

            data = data.transpose([0,1,3,2])

            if ((data.shape[1] != 3) or (data.shape[2] != 3)):
                logging.error("couldn't load file: invalid data shape " + str(data.shape))
                return None

            return cls(dataset_type, data, data.shape[3])

        logging.error("couldn't load file")
        return None

    @classmethod
    def for_recording(cls, num_imus, max_samples):
        return cls('raw', numpy.zeros([num_imus, 3, 3, max_samples]), 0)

    @classmethod
    def from_data(cls, data_type, data):
        return cls(data_type, data, data.shape[3])


    def save_file(self, filename):
        with h5py.File(filename, 'w') as datafile:
            data_group = datafile.create_group('data')
            data_group.attrs['description'] = self.dataset_type
            for i in range(0, self.imu_data.shape[0]):
                imu_group = data_group.create_group('imu' + str(i))
                imu_group.create_dataset('accel', data=self.imu_data[i, Ic_data.ACCEL_INDEX, :, :].transpose())
                imu_group.create_dataset('gyro',  data=self.imu_data[i, Ic_data.GYRO_INDEX, :, :].transpose())
                imu_group.create_dataset('mag',   data=self.imu_data[i, Ic_data.MAG_INDEX, :, :].transpose())




    def set_dataset_type(self, data_type):
        self.dataset_type = data_type
        if(self.dataset_type == 'transformed'):
            self.accel_units_string = 'Dynamic acceleration (meters per second squared)'
            self.gyro_units_string  = 'Orientation (radians)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
        elif((self.dataset_type == 'raw') or (self.dataset_type == 'calibration')):
            self.accel_units_string = 'Acceleration (meters per second squared)'
            self.gyro_units_string  = 'Gyroscope (radians per second)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
        else:
            logging.error("invalid dataset type: " + dataset_type + "assuming raw data")
            self.accel_units_string = 'Acceleration (meters per second squared)'
            self.gyro_units_string  = 'Gyroscope (radians per second)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
            return False
        return True

    def trim_data(self):
        self.set_max_samples(self.num_samples)

    def set_max_samples(self, max_samples):
        self.mutex.lock()

        if (max_samples < self.imu_data.shape[3]):
            # SHRINK
            if (self.num_samples < max_samples):
                # THROW OUT EMPTY DATA
                self.imu_data = self.imu_data[:,:,:, :max_samples]
            else:
                # THROW OUT EMPTY DATA AND OLD DATA
                self.imu_data = self.imu_data[:,:,:, self.num_samples - max_samples : self.num_samples]
                self.num_samples = max_samples

        elif (max_samples > self.imu_data.shape[3]):
            # GROW
            self.imu_data = numpy.append(self.imu_data, numpy.zeros([self.num_imus, 3, 3, max_samples - self.imu_data.shape[3]]), 3)

        self.mutex.unlock()

    def has_data(self):
        return (self.num_samples > 0)


    def add_sample(self, sample):
        assert(len(sample) == self.num_imus)

        self.mutex.lock()

        if (self.num_samples == self.imu_data.shape[3]):
            self.imu_data = numpy.roll(self.imu_data, -1, 3)
        else:
            self.num_samples += 1

        self.imu_data[:,:,:,self.num_samples-1] = sample

        self.mutex.unlock()

        self.total_samples += 1




    ##################################################
    #            LOAD AND SAVE FILES                 #
    ##################################################


    def complex_load_hdf5_file(self, filename, root_group='/data', accelerometer_group='accel', gyroscope_group='gyro',
                       magnetometer_group='mag', time_group='time'):
        with h5py.File(filename, 'r') as datafile:
            if datafile is None:
                logging.debug('datafile is None')
                return False

            timepath = '/'.join([root_group, time_group])
            self.imu_data = {}

            #if timepath in datafile:
            #    self.imu_data['timestamps'] = datafile.get(timepath)[()]
            #else:
            #    return False

            self.imu_data['imus'] = []

            i = 0
            done = False
            shapewarning = False

            self.num_samples = None
            while not done:
                ext = str(i + 1)

                accelpath = '/'.join([root_group, accelerometer_group])
                gyropath = '/'.join([root_group, gyroscope_group])
                magpath = '/'.join([root_group, magnetometer_group])

                a1 = datafile.get(accelpath + ext)
                if (i == 0) and (a1 is None):
                    a1 = datafile.get(accelpath)
                if a1 is not None:
                    a1 = numpy.array(a1)
                    if a1.shape[0] == 3:
                        shapewarning = True
                        a1 = a1.T
                    elif a1.shape[1] != 3:
                        raise ValueError("Accelerometer data does not have the right structure (should be Nx3)")
                    a1 = a1.T.tolist()

                g1 = datafile.get(gyropath + ext)
                if (i == 0) and (g1 is None):
                    g1 = datafile.get(gyropath)
                if g1 is not None:
                    g1 = numpy.array(g1)
                    if g1.shape[0] == 3:
                        shapewarning = True
                        g1 = g1.T
                    elif g1.shape[1] != 3:
                        raise ValueError("Gyroscope data does not have the right structure (should be Nx3)")
                    g1 = g1.T.tolist()

                m1 = datafile.get(magpath + ext)
                if (i == 0) and (m1 is None):
                    m1 = datafile.get(magpath)
                if m1 is not None:
                    m1 = numpy.array(m1)
                    if m1.shape[0] == 3:
                        shapewarning = True
                        m1 = m1.T
                    elif m1.shape[1] != 3:
                        raise ValueError("Magnetometer data does not have the right structure (should be Nx3)")
                    m1 = m1.T.tolist()

                if a1 is None and g1 is None:
                    done = True
                else:
                    self.imu_data['imus'].append({})
                    self.imu_data['imus'][i]['accel'] = a1
                    self.imu_data['imus'][i]['gyro'] = g1
                    self.imu_data['imus'][i]['mag'] = m1
                    i += 1

            if shapewarning:
                logging.warning("Data is shaped incorrectly (should be Nx3). Attempting to load anyway")

            self.num_imus = i
            logging.debug('Loaded {} IMUs'.format(self.num_imus))

            if (Ic_global.USE_ENCODER) and '/data/Encoder' in datafile:
                self.imu_data['encoder'] = datafile.get('data/Encoder')[()]

            self.num_samples = len(self.imu_data['imus'][0]['accel'][0])
            for imu in range(0, self.num_imus):
                for mode in (['accel', 'gyro', 'mag']):
                    for axis in range(0, 3):
                        if (self.num_samples != len(self.imu_data['imus'][imu][mode][axis])):
                            logging.error("data loaded with uneven lengths")
                            return False

            return True
        return False

    


