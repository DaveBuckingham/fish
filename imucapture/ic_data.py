import h5py
import csv

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


    timestamp_signal = PyQt5.QtCore.pyqtSignal(float)


    def __init__(self):
        super(Ic_data, self).__init__()

        self.imu_data = {}
        self.imu_data['timestamps'] = collections.deque()

        self.data_lock = [False]

        self.saved = True

        self.total_samples = 0
        

    def has_data(self):
        return (len(self.imu_data['timestamps']) > 0)

    def num_samples(self):
        return len(self.imu_data['timestamps'])


    def reset_data(self, num_imus):
        # DICTIONARY OF LISTS AND OF LISTS OF DICTIONARIES OF LISTS OF LISTS
        self.data_lock[0] = True
        self.imu_data = {}
        self.imu_data['timestamps'] = collections.deque()

        # THIS DOESN'T WORK! * OPERATOR DOESN'T CREATE SEPARATE OBJECTS
        #self.imu_data['imus'] = [{'accel': [[],[],[]], 'gyro': [[],[],[]], 'mag': [[],[],[]]}] * num_imus

        # THIS WORKS
        self.imu_data['imus'] = [{'accel': [collections.deque(), collections.deque(), collections.deque()],
                                  'gyro':  [collections.deque(), collections.deque(), collections.deque()], 
                                  'mag':   [collections.deque(), collections.deque(), collections.deque()]
                                 }
                                 for i in range(num_imus)
                                ]


        if (Ic_global.USE_ENCODER):
            self.imu_data['encoder'] = []
        self.data_lock[0] = False
        self.num_imus = num_imus


    def add_sample(self, sample, limit):
        if (Ic_global.USE_ENCODER):
            assert(len(sample) == 3)
        else:
            assert(len(sample) == 2)
        assert(len(sample[1]) == self.num_imus)


        if(self.data_lock[0]):
            logging.info("WRITE LOCKED Ic_rx")
        else:
            self.data_lock[0] = True

            self.imu_data['timestamps'].append(sample[0])

            for i in (range(0, self.num_imus)):
                (self.imu_data['imus'][i]['accel'][0]).append(sample[1][i][0][0])
                (self.imu_data['imus'][i]['accel'][1]).append(sample[1][i][0][1])
                (self.imu_data['imus'][i]['accel'][2]).append(sample[1][i][0][2])

                (self.imu_data['imus'][i]['gyro'][0]).append(sample[1][i][1][0])
                (self.imu_data['imus'][i]['gyro'][1]).append(sample[1][i][1][1])
                (self.imu_data['imus'][i]['gyro'][2]).append(sample[1][i][1][2])

                (self.imu_data['imus'][i]['mag'][0]).append(sample[1][i][2][0])
                (self.imu_data['imus'][i]['mag'][1]).append(sample[1][i][2][1])
                (self.imu_data['imus'][i]['mag'][2]).append(sample[1][i][2][2])


            if (Ic_global.USE_ENCODER):
                self.imu_data['encoder'].append(sample[2])


            while (len(self.imu_data['timestamps']) > limit):
                self.imu_data['timestamps'].popleft()

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


                if (Ic_global.USE_ENCODER):
                    self.imu_data['encoder'].popleft()




            self.total_samples += 1
            self.data_lock[0] = False



    def as_list_of_triples(self, imu_index, mode):
        return list(zip(*self.imu_data['imus'][imu_index][mode]))


    ##################################################
    #            LOAD AND SAVE FILES                 #
    ##################################################

    def load_csv_file(self, filename):

        if (Ic_global.USE_ENCODER):
            expected_non_imu_columns = 2
        else:
            expected_non_imu_columns = 1

        self.num_imus = None
        with open(filename, 'r') as datafile:
            reader = csv.reader(datafile, delimiter=',')
            for row in reader:
                if (len(row) % 9 != expected_non_imu_columns):
                    return False
                row_num_imus = (len(row) - expected_non_imu_columns) // 9       # // for integer division in python3
                if self.num_imus is None:       # READING FIRST LINE OF CSV
                    self.num_imus = row_num_imus
                    self.reset_data(row_num_imus)
                else:
                    if (len(row) != (self.num_imus * 9) + expected_non_imu_columns):
                        return False
                row = list(map(lambda x: float(x) if ('.' in x) else int(x), row))
                self.imu_data['timestamps'].append(row[0])

                j = 1
                for i in range(0, row_num_imus):
                    self.imu_data['imus'][i]['accel'][0].append(row[j])
                    self.imu_data['imus'][i]['accel'][1].append(row[j+1])
                    self.imu_data['imus'][i]['accel'][2].append(row[j+2])
                    self.imu_data['imus'][i]['gyro'][0].append(row[j+3])
                    self.imu_data['imus'][i]['gyro'][1].append(row[j+4])
                    self.imu_data['imus'][i]['gyro'][2].append(row[j+5])
                    self.imu_data['imus'][i]['mag'][0].append(row[j+6])
                    self.imu_data['imus'][i]['mag'][1].append(row[j+7])
                    self.imu_data['imus'][i]['mag'][2].append(row[j+8])
                    j+=9

                if (Ic_global.USE_ENCODER):
                    self.imu_data['encoder'].append(row[-1])
            return True
        return False


    def load_hdf5_file(self, filename):
        with h5py.File(filename, 'r') as datafile:
            self.imu_data = {}
            self.imu_data['timestamps'] = datafile.get('data/time')[()]
            self.imu_data['imus'] = []

            i = 0
            ext = str(i + 1)
            while ('data/accel' + ext in datafile and 'data/gyro' + ext in datafile and 'data/mag' + ext in datafile):
                self.imu_data['imus'].append({})
                self.imu_data['imus'][i]['accel'] = list(map(list, zip(*datafile.get('data/accel' + ext)[()])))
                self.imu_data['imus'][i]['gyro'] = list(map(list, zip(*datafile.get('data/gyro' + ext)[()])))
                self.imu_data['imus'][i]['mag'] = list(map(list, zip(*datafile.get('data/mag' + ext)[()])))
                i += 1
                ext = str(i + 1)

            self.num_imus = i

            if (Ic_global.USE_ENCODER):
                self.imu_data['encoder'] = datafile.get('data/Encoder')[()]
            return True
        return False


    def save_hdf5_file(self, filename):
        with h5py.File(filename, 'w') as datafile:
            save_data = datafile.create_group("data")

            # save_data.create_dataset('t', data=self.imu_data['timestamps'])
            save_data.create_dataset('time', data=self.imu_data['timestamps'])

            for i in range(0, len(self.imu_data['imus'])):
                imu = self.imu_data['imus'][i]

                extension = str(i + 1)

                save_data.create_dataset('accel' + extension, data=self.as_list_of_triples(i, 'accel'))
                save_data.create_dataset('gyro' + extension, data=self.as_list_of_triples(i, 'gyro'))
                save_data.create_dataset('mag' + extension, data=self.as_list_of_triples(i, 'mag'))

            if (Ic_global.USE_ENCODER):
                save_data.create_dataset('Encoder', data=self.imu_data['encoder'])


    def save_csv_file(self, filename):
        with open(filename, 'w') as datafile:
            writer = csv.writer(datafile, delimiter=',')

            for i in range(0, len(self.imu_data['timestamps'])):
                row = [self.imu_data['timestamps'][i]]
                for j in range(0, len(self.imu_data['imus'])):
                    row.append(self.imu_data['imus'][j]['accel'][0][i])
                    row.append(self.imu_data['imus'][j]['accel'][1][i])
                    row.append(self.imu_data['imus'][j]['accel'][2][i])
                    row.append(self.imu_data['imus'][j]['gyro'][0][i])
                    row.append(self.imu_data['imus'][j]['gyro'][1][i])
                    row.append(self.imu_data['imus'][j]['gyro'][2][i])
                    row.append(self.imu_data['imus'][j]['mag'][0][i])
                    row.append(self.imu_data['imus'][j]['mag'][1][i])
                    row.append(self.imu_data['imus'][j]['mag'][2][i])

                if (Ic_global.USE_ENCODER):
                    row.append(self.imu_data['encoder'][i])
                writer.writerow(row)

