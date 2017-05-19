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
        gyro_min = self.data.imu_data['gyro'][start:end].min
        gyro_max = self.data.imu_data['gyro'][start:end].max
        accel_min = self.data.imu_data['accel'][start:end].min
        accel_max = self.data.imu_data['accel'][start:end].max
        gyro_ok = max(abs(gyro_min), abs(gyro_max)) < self.CALIBRATE_GYRO_THRESHOLD
        accel_ok = accel_max - accel_min < self.CALIBRself.ATE_ACCEL_DELTA_THRESHOLD
        return (gyro_ok and accel_ok)


    def get_calib_values(self, data):
        intervals = []
        start = 0
        end = self.HOLD_MIN
        while end <= data.num_samples():

            if (within_thresholds(start,end)):
                while (within_thresholds(start, end) and (end < data.num_samples())):
                    end += 1
                push(intervals, (start, end))
                start = end
                end = start + self.HOLD_MIN
            else:
                start += 1
                end += 1

        return intervals



