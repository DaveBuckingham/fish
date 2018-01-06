import h5py
import csv
import logging

import numpy

import PyQt5.QtCore

from imucapture.ic_global import *

import collections


try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_data(PyQt5.QtCore.QObject):


    mag_asas = []

    recording_signal = PyQt5.QtCore.pyqtSignal()


    data_frequency = None


    def __init__(self, processed=False):
        super(Ic_data, self).__init__()

        self.imu_data = {}

        self.data_lock = [False]

        self.total_samples = 0

        if(processed):
            self.data_description_string = 'filtered data'
            self.accel_units_string = 'Dynamic acceleration (meters per second squared)'
            self.gyro_units_string  = 'Orientation (radians)'
            self.mag_units_string   = 'Magnetometer (microteslas)'
        else:
            self.data_description_string = 'raw data'
            self.accel_units_string = 'Acceleration (meters per second squared)'
            self.gyro_units_string  = 'Gyroscope (radians per second)'
            self.mag_units_string   = 'Magnetometer (microteslas)'

        
        #self.num_imus = 0
        #self.num_samples = 0
        self.reset_data(0)

    def has_data(self):
         return (self.num_samples > 0)


    def reset_data(self, num_imus):
        #self.has_data = False
        self.num_samples = 0
        self.num_imus = num_imus

        # DICTIONARY OF LISTS AND OF LISTS OF DICTIONARIES OF LISTS OF LISTS
        self.data_lock[0] = True
        self.imu_data = {}

        # THIS WORKS
        self.imu_data['imus'] = [{'accel': [collections.deque(), collections.deque(), collections.deque()],
                                  'gyro':  [collections.deque(), collections.deque(), collections.deque()], 
                                  'mag':   [collections.deque(), collections.deque(), collections.deque()]
                                 }
                                 for i in range(self.num_imus)
                                ]

        # Q: Why don't you use numpy arrays?  It'll make the math *way* faster, and probably pyqtgraph too

        if (Ic_global.USE_ENCODER):
            self.imu_data['encoder'] = []
        self.data_lock[0] = False


    def add_sample(self, sample, limit='inf'):
        if (Ic_global.USE_ENCODER):
            assert(len(sample) == self.num_imus + 1)
        else:
            assert(len(sample) == self.num_imus)

        if(self.data_lock[0]):
            logging.info("WRITE LOCKED Ic_rx")
        else:
            # Q: are you aiming for a real thread safe lock here? This might not work
            self.data_lock[0] = True

            #self.imu_data['timestamps'].append(sample[0])

            # Q: If appending data is the reason not to use numpy arrays, could collect preallocate blocks and assign
            self.num_samples += 1
            for i in (range(0, self.num_imus)):

                (self.imu_data['imus'][i]['accel'][0]).append(sample[i][0][0])
                (self.imu_data['imus'][i]['accel'][1]).append(sample[i][0][1])
                (self.imu_data['imus'][i]['accel'][2]).append(sample[i][0][2])

                (self.imu_data['imus'][i]['gyro'][0]).append(sample[i][1][0])
                (self.imu_data['imus'][i]['gyro'][1]).append(sample[i][1][1])
                (self.imu_data['imus'][i]['gyro'][2]).append(sample[i][1][2])

                (self.imu_data['imus'][i]['mag'][0]).append(sample[i][2][0])
                (self.imu_data['imus'][i]['mag'][1]).append(sample[i][2][1])
                (self.imu_data['imus'][i]['mag'][2]).append(sample[i][2][2])


            if (Ic_global.USE_ENCODER):
                self.imu_data['encoder'].append(sample[-1])


            while (self.num_samples > limit):

                for i in (range(0, self.num_imus)):
                    self.imu_data['imus'][i]['accel'][0].popleft()
                    self.imu_data['imus'][i]['accel'][1].popleft()
                    self.imu_data['imus'][i]['accel'][2].popleft()

                    self.imu_data['imus'][i]['gyro'][0].popleft()
                    self.imu_data['imus'][i]['gyro'][1].popleft()
                    self.imu_data['imus'][i]['gyro'][2].popleft()

                    self.imu_data['imus'][i]['mag'][0].popleft()
                    self.imu_data['imus'][i]['mag'][1].popleft()
                    self.imu_data['imus'][i]['mag'][2].popleft()

                self.num_samples -= 1


                if (Ic_global.USE_ENCODER):
                    self.imu_data['encoder'].popleft()




            self.total_samples += 1
            self.data_lock[0] = False



    def as_list_of_triples(self, imu_index, mode):
        return list(zip(*self.imu_data['imus'][imu_index][mode]))


    ##################################################
    #            LOAD AND SAVE FILES                 #
    ##################################################


    def load_hdf5_file(self, filename, root_group='/data', accelerometer_group='accel', gyroscope_group='gyro',
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

            self.num_samples = None
            while not done:
                ext = str(i + 1)

                accelpath = '/'.join([root_group, accelerometer_group])
                gyropath = '/'.join([root_group, gyroscope_group])
                magpath = '/'.join([root_group, magnetometer_group])

                a1, g1, m1 = None, None, None
                if accelpath + ext in datafile:
                    a1 = list(map(list, zip(*datafile.get(accelpath + ext)[()])))
                elif (i == 0) and (accelpath in datafile):              # for the first IMU, the path may not have a number
                    a1 = list(map(list, zip(*datafile.get(accelpath)[()])))

                if gyropath + ext in datafile:
                    g1 = list(map(list, zip(*datafile.get(gyropath + ext)[()])))
                elif (i == 0) and (gyropath in datafile):              # for the first IMU, the path may not have a number
                    g1 = list(map(list, zip(*datafile.get(gyropath)[()])))

                if magpath + ext in datafile:
                    m1 = list(map(list, zip(*datafile.get(magpath + ext)[()])))
                elif (i == 0) and (magpath in datafile):              # for the first IMU, the path may not have a number
                    m1 = list(map(list, zip(*datafile.get(magpath)[()])))

                if a1 is None and g1 is None:
                    done = True
                else:
                    self.imu_data['imus'].append({})
                    self.imu_data['imus'][i]['accel'] = a1
                    self.imu_data['imus'][i]['gyro'] = g1
                    self.imu_data['imus'][i]['mag'] = m1
                    i += 1

            self.num_imus = i
            logging.debug('Loaded {} IMUs'.format(self.num_imus))

            if (Ic_global.USE_ENCODER):
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

    
    def simple_load_hdf5_file(self, filename):
        with h5py.File(filename, 'r') as datafile:
            if datafile is None:
                logging.debug('datafile is None')
                return False

            timepath = '/'.join([root_group, time_group])
            self.imu_data = {}
            #self.imu_data['timestamps'] = datafile.get('time')[()]
            self.imu_data['imus'] = []

            i = 0
            ext = str(i + 1)
            while ('imu' + ext in datafile):
                self.imu_data['imus'].append({})
                self.imu_data['imus'][i]['accel'] = list(map(list, zip(*datafile.get(('imu' + ext)/accel)[()])))
                self.imu_data['imus'][i]['gyro'] = list(map(list, zip(*datafile.get(('imu' + ext)/gyro)[()])))
                self.imu_data['imus'][i]['mag'] = list(map(list, zip(*datafile.get(('imu' + ext)/mag)[()])))
                i += 1
                ext = str(i + 1)

            self.data_description_string = datafile.attrs['description']
            self.accel_units_string      = accel_dataset.attrs['description']
            self.gyro_units_string       = gyro_dataset.attrs['description']
            self.mag_units_string        = mag_dataset.attrs['description']

            self.num_imus = i
            logging.debug('Loaded {} IMUs'.format(self.num_imus))

            if (Ic_global.USE_ENCODER):
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






    def save_hdf5_file(self, filename):
        with h5py.File(filename, 'w') as datafile:
            save_data = datafile.create_group("data")

            #save_data.create_dataset('time', data=self.imu_data['timestamps'])
            save_data.create_dataset('time', data=range(0, self.num_samples, Ic_global.MS_PER_SAMPLE))

            for i in range(0, len(self.imu_data['imus'])):
                imu = self.imu_data['imus'][i]

                extension = str(i + 1)

                save_data.create_dataset('accel' + extension, data=self.as_list_of_triples(i, 'accel'))
                save_data.create_dataset('gyro' + extension, data=self.as_list_of_triples(i, 'gyro'))
                save_data.create_dataset('mag' + extension, data=self.as_list_of_triples(i, 'mag'))

            if (Ic_global.USE_ENCODER):
                save_data.create_dataset('Encoder', data=self.imu_data['encoder'])




#    def rotate(self, R, imunum=0):
#        acc = self.as_list_of_triples(imunum, 'accel')
#        gyro = self.as_list_of_triples(imunum, 'gyro')
#        mag = self.as_list_of_triples(imunum, 'mag')
#
#        accr = [[],[],[]]
#        gyror = [[],[],[]]
#        magr = [[],[],[]]
#        for a, g, m in zip(acc, gyro, mag):
#            a1 = R.dot(numpy.array(a))
#            accr[0].append(a1[0])
#            accr[1].append(a1[1])
#            accr[2].append(a1[2])
#
#            g1 = R.dot(numpy.array(g))
#            gyror[0].append(g1[0])
#            gyror[1].append(g1[1])
#            gyror[2].append(g1[2])
#
#            m1 = R.dot(numpy.array(m))
#            magr[0].append(m1[0])
#            magr[1].append(m1[1])
#            magr[2].append(m1[2])
#
#        self.imu_data['imus'][imunum]['accel'] = accr
#        self.imu_data['imus'][imunum]['gyro'] = gyror
#        self.imu_data['imus'][imunum]['mag'] = magr
