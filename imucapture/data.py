import h5py
import csv
import logging

import sys

import numpy
import math

import time

import PyQt5.QtCore

from imucapture.global_data import Global_data


try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Data(PyQt5.QtCore.QObject):


    ACCEL_INDEX = 0
    GYRO_INDEX  = 1
    MAG_INDEX   = 2

    def __init__(self, dataset_type, data, num_samples, timestamps=None):
        super(Data, self).__init__()

        self.mutex = PyQt5.QtCore.QMutex()

        self.set_dataset_type(dataset_type)

        self.imu_data = data

        self.utc_system_time_at_trigger = ""

        self.trigger_delay = 0

        self.num_imus = self.imu_data.shape[0]
        self.num_samples = num_samples
        self.total_samples = self.num_samples
        self.timestamps = timestamps


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
    def from_file_smart(cls, filename, resample=False, resample_rate=200.0):

        start = time.time()

        with h5py.File(filename, 'r') as datafile:
            if datafile is None:
                logging.error("couldn't load file")
                return None


            if ('data' in datafile):
                root = datafile['data']
            elif ('Data' in datafile):
                root = datafile['Data']
            else:
                root = datafile

            is_numbered_data = False
            if ('imu0' in root):
                no_imu_group = False
            else:
                if ('accel1' in root):
                    is_numbered_data = True
                    no_imu_group = False
                else:
                    no_imu_group = True

            data = None
            imu_index = 0
            while (no_imu_group or ('imu' + str(imu_index) in root) or \
                           (is_numbered_data and ('accel' + str(imu_index+1) in root))):

                if (no_imu_group or is_numbered_data):
                    imu = root
                else:
                    imu = root['imu' + str(imu_index)]

                no_imu_group = False

                if is_numbered_data and ('accel' + str(imu_index+1) in imu):
                    accel_data = imu['accel' + str(imu_index+1)]
                elif ('accel' in imu):
                    accel_data = imu['accel']
                elif ('Accel' in imu):
                    accel_data = imu['Accel']
                elif ('acceleration' in imu):
                    accel_data = imu['acceleration']
                elif ('Acceleleration' in imu):
                    accel_data = imu['Acceleration']
                else:
                    logging.error("couldn't load file: accel data not found")
                    return None

                if is_numbered_data and ('gyro' + str(imu_index+1) in imu):
                    gyro_data = imu['gyro' + str(imu_index+1)]
                elif ('gyro' in imu):
                    gyro_data = imu['gyro']
                elif ('Gyro' in imu):
                    gyro_data = imu['Gyro']
                elif ('gyroscope' in imu):
                    gyro_data = imu['gyroscope']
                elif ('Gyroscope' in imu):
                    gyro_data = imu['Gyroscope']
                else:
                    logging.error("couldn't load file: gyro data not found")
                    return None

                if is_numbered_data and ('mag' + str(imu_index+1) in imu):
                    mag_data = imu['mag' + str(imu_index+1)]
                elif ('mag' in imu):
                    mag_data = imu['mag']
                elif ('Mag' in imu):
                    mag_data = imu['Mag']
                elif ('magnetometer' in imu):
                    mag_data = imu['magnetometer']
                elif ('Magnetometer' in imu):
                    mag_data = imu['Magnetometer']
                else:
                    mag_data = numpy.zeros_like(accel_data)

                accel_data = accel_data[()]
                gyro_data = gyro_data[()]
                mag_data = mag_data[()]

                if (accel_data.shape[1] == 3):
                    accel_data = accel_data.transpose([1,0])
                if (gyro_data.shape[1] == 3):
                    gyro_data = gyro_data.transpose([1,0])
                if (mag_data.shape[1] == 3):
                    mag_data = mag_data.transpose([1,0])

                if (data is None):
                    data = numpy.array([numpy.stack((accel_data, gyro_data, mag_data))])
                else:
                    data1 = numpy.stack((accel_data, gyro_data, mag_data))
                    data = numpy.append(data, data1[numpy.newaxis, ...], 0)

                imu_index += 1

            timestamps = None
            timename = None
            if ('t' in root):
                timestamps = root['t'][()]
                timename = 't'
            elif ('time' in root):
                timestamps = root['time'][()]
                timename = 'time'

            if (timestamps is not None) and resample:
                # REMOVE LAST ENTRY IF TIMESTAMP == 0
                if (timestamps[-1] == 0):
                    timestamps = numpy.delete(timestamps, -1, 0)
                    data = numpy.delete(data, -1, 3)

                # REMOVE ALL ENTRIES WITH TIMESTAMP == 0
                # sample_index = timestamps.size - 1
                # while sample_index >= 0.0:
                #     if (timestamps[sample_index] == 0.0):
                #         timestamps = numpy.delete(timestamps, sample_index, 0)
                #         accel_data = numpy.delete(accel_data, sample_index, 1)
                #         gyro_data = numpy.delete(gyro_data, sample_index, 1)
                #         mag_data = numpy.delete(mag_data, sample_index, 1)
                #     sample_index -= 1

                if (any(x>=y for x, y in zip(timestamps, timestamps[1:]))):
                    logging.error("couldn't load file: non-monotonic timestamps")
                    return None

                # START TIMESTAMPS AT 0 AND CONVERT FROM NANOSECONDS TO MILLISECONDS
                tscale = 1
                if 'Units' in root[timename].attrs:
                    tunit = ''.join([chr(c) for c in root[timename].attrs['Units']])
                    if tunit[:4] == 'nsec':
                        tscale = 1e6
                    elif tunit[:4] == 'msec':
                        tscale = 1

                start_time = timestamps[0]
                timestamps[:] = [(x - start_time) / tscale for x in timestamps]


                # TIMESTAMPS FOR PERFECT 200 HZ RECORDINGS
                dt = 1000.0 / resample_rate
                desired_timestamps = numpy.arange(0, timestamps[-1], dt)

                interpolated_data = numpy.empty(data.shape[0:-1] + (desired_timestamps.size,))
                for imu1 in range(data.shape[0]):
                    for ax1 in range(3):
                        for ax2 in range(3):
                            interpolated_data[imu1, ax1, ax2, :] = numpy.interp(desired_timestamps, timestamps, data[imu1, ax1, ax2, :])
                timestamps = desired_timestamps
                data = interpolated_data

            if ('description' in root.attrs):
                dataset_type = root.attrs['description']
            else:
                dataset_type = 'unknown'

            if ((data.shape[1] != 3) or (data.shape[2] != 3)):
                logging.error("couldn't load file: invalid data shape " + str(data.shape))
                return None

            return cls(dataset_type, data, data.shape[3], timestamps)

        logging.error("couldn't load file")
        return None






    @classmethod
    def for_recording(cls, num_imus, max_samples):
        return cls('raw', numpy.zeros([num_imus, 3, 3, max_samples]), 0)

    @classmethod
    def from_data(cls, data_type, data, timestamps):
        return cls(data_type, data, data.shape[3], timestamps)


    def save_file(self, filename):
        with h5py.File(filename, 'w') as datafile:
            data_group = datafile.create_group('data')
            data_group.attrs['description'] = self.dataset_type
            data_group.attrs['trigger_time'] = self.utc_system_time_at_trigger
            data_group.attrs['sample_rate_hz'] = Global_data.SAMPLE_FREQ_HZ
            data_group.attrs['trigger_delay_samples'] = self.trigger_delay
            for i in range(0, self.imu_data.shape[0]):
                imu_group = data_group.create_group('imu' + str(i))
                imu_group.create_dataset('accel', data=self.imu_data[i, Data.ACCEL_INDEX, :, :].transpose())
                imu_group.create_dataset('gyro',  data=self.imu_data[i, Data.GYRO_INDEX, :, :].transpose())
                imu_group.create_dataset('mag',   data=self.imu_data[i, Data.MAG_INDEX, :, :].transpose())
                data_group.create_dataset('time', data=list(range(0,self.num_samples*Global_data.MS_PER_SAMPLE,Global_data.MS_PER_SAMPLE)))




    def set_dataset_type(self, data_type):
        if(data_type == 'transformed'):
            self.dataset_type = data_type
            self.accel_units_string = 'Dynamic acceleration (meters per second squared)'
            self.gyro_units_string  = 'Orientation (radians)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
        elif(data_type == 'raw'):
            self.dataset_type = data_type
            self.accel_units_string = 'Acceleration (meters per second squared)'
            self.gyro_units_string  = 'Gyroscope (radians per second)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
        else:
            logging.error("unknown dataset type: " + data_type + "assuming raw data")
            self.dataset_type = 'raw'
            self.accel_units_string = 'Acceleration (meters per second squared)'
            self.gyro_units_string  = 'Gyroscope (radians per second)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
            return False
        return True

    def trim_data(self):
        self.set_max_samples(self.num_samples)

    def get_one_imu(self, imunum):
        return Data.from_data(self.dataset_type, self.imu_data[imunum:imunum+1, ...], self.timestamps)

    def get_acceleration(self, imunum=0):
        return self.imu_data[imunum, Data.ACCEL_INDEX, ...]

    def get_gyroscope(self, imunum=0):
        return self.imu_data[imunum, Data.GYRO_INDEX, ...]

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

            if (Global_data.USE_ENCODER) and '/data/Encoder' in datafile:
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

    


