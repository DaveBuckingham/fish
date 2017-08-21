import numpy
import math
from scipy import interpolate, signal
from copy import copy

import itertools
import logging



class Am_get_basis(object):

    def __init__(self):
        self.MAX_INTERVALS = 5


        # THESE VALUES WORK WITH MEASUREMENTS TAKEN AT DESK WITH CHIP ON A BOARD
        #self.HOLD_MIN = 800               # minimum window size. 800 samples ~ 4 seconds
        #self.GYRO_THRESHOLD = 0.1         # max absolute value in window. degrees per second.
        #self.ACCEL_DELTA_THRESHOLD = 0.5  # max difference in window. meters per second squared.
        #self.ORTHOGONAL_THRESHOLD = 0.1   # max distance from right angle (in radians) between two vectors. 0.1 rad ~ 5.7 degs.

        # TRIPPLE ALL THE THRESHOLDS TO BE MORE FORGIVING WHEN CHIP IS ON A FISH
        self.HOLD_MIN = 600               # minimum window size. 600 samples ~ 3 seconds
        self.GYRO_THRESHOLD = 0.6
        self.ACCEL_DELTA_THRESHOLD = 1.5
        self.ORTHOGONAL_THRESHOLD = 0.3


    # ARE TWO 3D VECTORS ORTHOGONAL?
    def _orthogonal(self, vec1, vec2, threshold=None):
        if threshold is None:
            threshold = self.ORTHOGONAL_THRESHOLD
        dot_product = (vec1[0] * vec2[0]) + (vec1[1] * vec2[1]) + (vec1[2] * vec2[2])
        len1 = math.sqrt(vec1[0]**2 + vec1[1]**2 + vec1[2]**2)
        len2 = math.sqrt(vec2[0]**2 + vec2[1]**2 + vec2[2]**2)
        angle_between_vectors = math.acos(dot_product / (len1 * len2))  # RADIANS
        error = abs(1.5 - angle_between_vectors)                        # DISTANCE FROM RIGHT ANGLE
        return (error <= threshold)

    def _find_orthogonal_triple(self, vectors):
        triples = []
        for v1, v2, v3 in itertools.combinations(vectors, 3):
            if(self._orthogonal(v1, v2) and self._orthogonal(v1, v3) and self._orthogonal(v2, v3)):
                triples.append((v1, v2, v3))
        if(len(triples) > 1):
            logging.error("found more than 1 triple of orthogonal vectors")
            return(None)
        elif(len(triples) < 1):
            logging.error("could not find 3 orthogonal vectors")
            return(None)
        else:
            return(triples[0])

    def _calib_within_thresholds(self, data, start, end):
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
        return True


    def _mean(self, list):
        return (sum(list) / float(len(list)))


    def _gram_schmidt(self, U):
        k = U.shape[1]
        assert(k == 3)
        V = copy(U)

        for i in range(k):
            V[:, i] /= numpy.linalg.norm(V[:, i])
            for j in range(i+1, k):
                proj = numpy.dot(V[:, i], V[:, j]) * V[:, i]
                V[:, j] -= proj

        return V

    def _gram_schmidt2(self, X):
        Q, R = numpy.linalg.qr(X)
        return -Q

    def get_intervals(self, data):
        intervals = []
        mean_accels = []
        start = 0
        end = self.HOLD_MIN
        while end <= data.num_samples():

            if (self._calib_within_thresholds(data, start, end)):
                while (self._calib_within_thresholds(data, start, end) and (end < data.num_samples())):
                    end += 1
                intervals.append((start, end))
                start = end
                end = start + self.HOLD_MIN
            else:
                start += 1
                end += 1

        logging.info("found " + str(len(intervals)) + " steady intervals:")
        logging.info(intervals)

        return intervals


    def get_basis_vector(self, data, intervals=None):
        if (intervals is None):
            intervals = self.get_intervals(data)

        if(len(intervals) < 3):
            logging.error("fewer than 3 steady intervals")
            return None


        # SET AN ARBITRATY LIMIT, IF TOO LARGE, find_orthogonal_triple() WILL BE SLOW
        if(len(intervals) > self.MAX_INTERVALS):
            logging.error("too many steady intervals, the limit is " + str(self.MAX_INTERVALS))
            return None


        means = []
        for start, end in intervals:
            for imu in range(0, data.num_imus):
                mean_x = self._mean(data.imu_data['imus'][imu]['accel'][0][start:end])
                mean_y = self._mean(data.imu_data['imus'][imu]['accel'][1][start:end])
                mean_z = self._mean(data.imu_data['imus'][imu]['accel'][2][start:end])
                means.append((mean_x, mean_y, mean_z))
                
        original = self._find_orthogonal_triple(means)

        if original is None:
            return None

        logging.info("found approximately orthogonal vectors:\n" + str(original[0]) + "\n" + str(original[1]) + "\n" + str(original[2]))
        logging.info("applying Gram-Schmidt")

        # TAKE SOME APPROXMATELY ORTHOGONAL VECTORS AND MAKE THEM EXACTLY ORTHOGONAL
        # ALSO NORMALIZE
        orthonormal = self._gram_schmidt(numpy.asarray(original))

        # MAKE SURE THE BASIS IS RIGHT-HANDED
        if(numpy.dot(numpy.cross(orthonormal[0], orthonormal[1]), orthonormal[2]) < 0):
            orthonormal[0,:], orthonormal[1,:] = orthonormal[1,:], orthonormal[0,:].copy()
            # OR
            # orthonormal[:, 2] = -orthonormal[:, 2]
        assert(numpy.dot(numpy.cross(orthonormal[:,0], orthonormal[:,1]), orthonormal[:,2]) > 0)

        as_tuple = tuple(map(tuple, orthonormal))
        return as_tuple


