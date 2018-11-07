import numpy
import math
from copy import copy

import itertools
import logging

from imucapture.data import Data



class Get_basis(object):

    def __init__(self):
        self.MAX_INTERVALS = 5


        #self.HOLD_MIN = 600               # minimum window size. 600 samples ~ 3 seconds
        #self.GYRO_THRESHOLD = 0.6         # max absolute value in window. degrees per second.
        #self.ACCEL_DELTA_THRESHOLD = 1.5  # max difference in window. meters per second squared.
        #self.ORTHOGONAL_THRESHOLD = 0.3   # max distance from right angle (in radians) between two vectors. 0.1 rad ~ 5.7 degs.

        # RELAX THRESHOLDS TO BE MORE FORGIVING WHEN CHIP IS ON A FISH
        self.HOLD_MIN = 600
        self.GYRO_THRESHOLD = 0.6
        self.ACCEL_DELTA_THRESHOLD = 1.5
        self.ORTHOGONAL_THRESHOLD = 0.7


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


    def _mean(self, list):
        return (sum(list) / float(len(list)))


    def _gram_schmidt(self, U):
        k = U.shape[1]
        assert(k == 3)
        V = copy(U)

        for i in range(k):
            V[:, i] /= numpy.linalg.norm(V[:, i])

        for i in range(k):
            V[:, i] /= numpy.linalg.norm(V[:, i])
            for j in range(i+1, k):
                proj = numpy.dot(V[:, i], V[:, j]) * V[:, i]
                V[:, j] -= proj

        return V

    def _gram_schmidt2(self, X):
        Q, R = numpy.linalg.qr(X)
        return -Q




    def _calib_within_thresholds(self, data, start, end):
        for imu in range(0, data.num_imus):
            for axis in ([0,1,2]):
                gyro_min = min(data.imu_data[imu, Data.GYRO_INDEX, axis, start:end])
                gyro_max = max(data.imu_data[imu, Data.GYRO_INDEX, axis, start:end])
                gyro_ok = (max(abs(gyro_min), abs(gyro_max)) < self.GYRO_THRESHOLD)

                accel_min = min(data.imu_data[imu, Data.ACCEL_INDEX, axis, start:end])
                accel_max = max(data.imu_data[imu, Data.ACCEL_INDEX, axis, start:end])
                accel_ok = (accel_max - accel_min < self.ACCEL_DELTA_THRESHOLD)

                if not (gyro_ok and accel_ok):
                    return False
        return True



    def get_intervals(self, data):
        intervals = []
        mean_accels = []
        start = 0
        end = self.HOLD_MIN

        while end <= data.num_samples:

            if (self._calib_within_thresholds(data, start, end)):
                while (self._calib_within_thresholds(data, start, end) and (end < data.num_samples)):
                    # THIS LOOP TAKES A LONG TIME
                    #end += 1

                    # A HACK TO SPEED IT UP...THERE HAS TO BE A BETTER WAY
                    end += 20
                    if (not(self._calib_within_thresholds(data, start, end) and (end < data.num_samples))):
                        end -= 19

                intervals.append((start, end))
                start = end
                end = start + self.HOLD_MIN
            else:
                start += 1
                end += 1

        logging.info("found " + str(len(intervals)) + " steady intervals:")
        logging.info(intervals)

        return intervals


    def get_bases(self, data, intervals):
        if(len(intervals) < 3):
            logging.error("fewer than 3 steady intervals")
            return None

        # SET AN ARBITRATY LIMIT, IF TOO LARGE, find_orthogonal_triple() WILL BE SLOW
        if(len(intervals) > self.MAX_INTERVALS):
            logging.error("too many steady intervals, the limit is " + str(self.MAX_INTERVALS))
            return None

        imu_bases = []
        for imu in range(0, data.num_imus):

            means = []
            for start, end in intervals:
                mean_x = self._mean(data.imu_data[imu, Data.ACCEL_INDEX, 0, start:end])
                mean_y = self._mean(data.imu_data[imu, Data.ACCEL_INDEX, 1, start:end])
                mean_z = self._mean(data.imu_data[imu, Data.ACCEL_INDEX, 2, start:end])
                means.append((mean_x, mean_y, mean_z))
                
            original = self._find_orthogonal_triple(means)

            if original is None:
                return None

            logging.info("found approximately orthogonal vectors for imu " + str(imu) + ":\n" + str(original[0]) + "\n" + str(original[1]) + "\n" + str(original[2]))
            logging.info("applying Gram-Schmidt")

            # TAKE SOME APPROXMATELY ORTHOGONAL VECTORS AND MAKE THEM EXACTLY ORTHOGONAL
            # ALSO NORMALIZE
            orthonormal = self._gram_schmidt(numpy.asarray(original))

            # MAKE SURE THE BASIS IS RIGHT-HANDED
            if(numpy.dot(numpy.cross(orthonormal[0], orthonormal[1]), orthonormal[2]) < 0):
                logging.debug('Basis is not right handed!  Flipping z axis')
                orthonormal[:, 2] = -orthonormal[:, 2]
            assert(numpy.dot(numpy.cross(orthonormal[:,0], orthonormal[:,1]), orthonormal[:,2]) > 0)

            # CAN SKIP TUPLE?
            as_tuple = tuple(map(tuple, orthonormal))
            imu_bases.append(numpy.array(as_tuple).transpose())

        return imu_bases


