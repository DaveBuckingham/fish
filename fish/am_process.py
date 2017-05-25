#import numpy as np
#from scipy import interpolate, signal
#import Quaternion
#import h5py
#from copy import copy
#import re
#import matplotlib.pyplot as plt

import itertools



class Am_process(object):

    def __init__(self):
        self.HOLD_MIN = 800                # 4 seconds
        self.GYRO_THRESHOLD = 20           # max +- in window
        self.ACCEL_DELTA_THRESHOLD = 2000   # max difference in window
        self.ORTHOGONAL_THRESHOLD = 20000000

    # ARE TWO 3D VECTORS ORTHOGONAL?
    def orthogonal(self, vec1, vec2):
        error = (vec1[0] * vec2[0]) + (vec1[1] * vec2[1]) + (vec1[2] * vec2[2])
        return (error < self.ORTHOGONAL_THRESHOLD)

    def find_orthogonal_triple(self, vectors):
        triples = []
        for v1, v2, v3 in itertools.combinations(vectors, 3):
            if(self.orthogonal(v1, v2) and self.orthogonal(v1, v3) and self.orthogonal(v2, v3)):
                triples.append((v1, v2, v3))
        if(len(triples) == 1):
            return(triples[0])
        else:
            return(None)

    def calib_within_thresholds(self, data, start, end):
        gyro_ok = True
        accel_ok = True
        for imu in range(0, data.num_imus):
            for axis in ([0,1,2]):
                gyro_min = min(data.imu_data['imus'][imu]['gyro'][axis][start:end])
                gyro_max = max(data.imu_data['imus'][imu]['gyro'][axis][start:end])
                gyro_ok = gyro_ok and (max(abs(gyro_min), abs(gyro_max)) < self.GYRO_THRESHOLD)

                accel_min = min(data.imu_data['imus'][imu]['accel'][axis][start:end])
                accel_max = max(data.imu_data['imus'][imu]['accel'][axis][start:end])
                accel_ok = accel_ok and (accel_max - accel_min < self.ACCEL_DELTA_THRESHOLD)

                if not (gyro_ok and accel_ok):
                    return False

        #print(str(gyro_ok) + "  " + str(accel_ok))
        return True
        #return (gyro_ok and accel_ok)


    def mean(self, list):
        return (sum(list) / float(len(list)))

    def get_intervals(self, data):
        intervals = []
        mean_accels = []
        start = 0
        end = self.HOLD_MIN
        while end <= data.num_samples():

            if (self.calib_within_thresholds(data, start, end)):
                while (self.calib_within_thresholds(data, start, end) and (end < data.num_samples())):
                    end += 1
                intervals.append((start, end))
                start = end
                end = start + self.HOLD_MIN
            else:
                start += 1
                end += 1
        return intervals


    def get_calib_values(self, data):
        intervals = self.get_intervals(data)

        # SET AN ARBITRATY LIMIT, IF TOO LARGE, find_orthogonal_triple() WILL BE SLOW
        if(len(intervals) > 5):
            return None

        means = []
        for start, end in intervals:
            for imu in range(0, data.num_imus):
                mean_x = self.mean(data.imu_data['imus'][imu]['accel'][0][start:end])
                mean_y = self.mean(data.imu_data['imus'][imu]['accel'][1][start:end])
                mean_z = self.mean(data.imu_data['imus'][imu]['accel'][2][start:end])
                means.append((mean_x, mean_y, mean_z))
                
        return self.find_orthogonal_triple(means)







