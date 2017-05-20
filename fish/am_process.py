#import numpy as np
#from scipy import interpolate, signal
#import Quaternion
#import h5py
#from copy import copy
#import re
#import matplotlib.pyplot as plt


class Am_process(object):

    def __init__(self):
        self.HOLD_MIN = 300                # 4 seconds
        self.GYRO_THRESHOLD = 100           # max +- in window
        self.ACCEL_DELTA_THRESHOLD = 1000   # max difference in window


    def calib_within_thresholds(self, data, start, end):
        gyro_ok = True
        accel_ok = True
        for i in range(0, data.num_imus):
            for axis in ([0,1,2]):
                gyro_min = min(data.imu_data['imus'][i]['gyro'][axis][start:end])
                gyro_max = max(data.imu_data['imus'][i]['gyro'][axis][start:end])
                gyro_ok = gyro_ok and (max(abs(gyro_min), abs(gyro_max)) < self.GYRO_THRESHOLD)

                accel_min = min(data.imu_data['imus'][i]['accel'][axis][start:end])
                accel_max = max(data.imu_data['imus'][i]['accel'][axis][start:end])
                accel_ok = accel_ok and (accel_max - accel_min < self.ACCEL_DELTA_THRESHOLD)

                if not (gyro_ok and accel_ok):
                    return False

        #print(str(gyro_ok) + "  " + str(accel_ok))
        return True
        #return (gyro_ok and accel_ok)


    def get_calib_values(self, data):
        intervals = []
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



