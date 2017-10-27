import numpy as np
from scipy import interpolate, signal
import quaternion
import h5py
from copy import copy
import re

import matplotlib.pyplot as plt

class IMU(object):
    """NB: Internal storage of time, acceleration, and gyro data is in sec, m/sec^2, and rad/sec"""

    def __init__(self):
        self.t = []
        self.gyro = []
        self.sampfreq = None

        self.Qbias = None
        self.Qgyro = None
        self.Qacc = None
        self.Qdyn = None

        self.gN = np.array([0, 0, 1])

    def set_gyro(self, t, gyro, resamplefreq=None):
        """Sets just the gyro field, potentially resampling. Just for testing"""
        if resamplefreq is not None:
            t, gyro = self._resample_gyro(t, gyro, resamplefreq)
            sampfreq = resamplefreq
        else:
            t, gyro = t, gyro
            sampfreq = np.mean(np.diff(t))

        self.sampfreq = sampfreq
        self.t = t
        self.gyro = self._filter_gyro(t, gyro)

    def load(self, filename, t_dataset='/data/t', gyro_dataset='/data/Gyro', accel_dataset='/data/Accel',
             resamplefreq=None, t_units='ms', acc_units='m/s^2', gyro_units='rad/s'):
        """Loads IMU data from an HDF5 file, potentially resampling.

        Arguments:
            filename - File to load data from
            t_dataset - Name of the dataset that has the time (default '/data/t')
            gyro_dataset - Name of the dataset with gyro data (default '/data/Gyro')
            accel_dataset - Name of the dataset with accelerometer data (default '/data/Accel')
            resamplefreq - If not None, resample data at a constant sampling frequency. Could be a number in Hz or
                'mean' to resample at the mean rate. (default None)
            t_units - Units of the time dataset. Will convert to seconds. (default 'ms')
            acc_units - Units of the acceleration dataset in the file. Will convert to m/s^2. (options are 'g' or
                'm/s^2'; default 'm/s^2')
            gyro_units - Units of the gyro dataset in the file. Will convert to rad/s. (options are 'deg/s' or
                'rad/s'; default 'rad/s')
        """

        with h5py.File(filename, 'r') as h5file:
            acc = np.array(h5file[accel_dataset])
            gyro = np.array(h5file[gyro_dataset])
            t0 = np.array(h5file[t_dataset]).astype(float)

        t0, acc, gyro = self._convert_units(t0, acc, gyro, t_units, acc_units, gyro_units)

        if resamplefreq is not None:
            if isinstance(resamplefreq, str):
                if resamplefreq == 'mean':
                    resamplefreq = np.around(1.0 / np.mean(np.diff(t0)), decimals=-1)
                else:
                    raise ValueError('Unrecognized resampling option: {}'.format(resamplefreq))

            self.t_irr = t0
            self.gyro_irr = gyro
            self.acc_irr = acc

            t = np.arange(t0[0], t0[-1], 1.0/resamplefreq)

            gyro = interpolate.interp1d(t0, gyro, axis=0)(t)
            acc = interpolate.interp1d(t0, acc, axis=0)(t)
            sampfreq = resamplefreq
        else:
            t = t0
            sampfreq = 1.0/np.mean(np.diff(t0))

        self.t0 = t
        self.t = t
        self.acc0 = acc
        self.acc = self.acc0
        self.gyro0 = gyro
        self.gyro = self.gyro0
        self.sampfreq = sampfreq

    def filter(self, order=None, gyro_cutoff=None, acc_cutoff=None, nsamp=None, method='butter'):
        """Filter IMU data using either a butterworth filter or a running mean.

        Cutoffs are given as real frequencies in Hz, not normalized frequencies.

        For the Butterworth filter, cutoffs can be specified as a single frequency, in which case the filter uses a low
        pass filter, or as two frequencies, in which case the filter is (essentially) a bandpass filter.

        Because IIR filters tend to perform poorly at really low frequencies, the bandpass first uses a running mean
        at the lowest frequency to get the low baseline, then subtracts that the from data, then runs a low pass filter
        at the higher frequency.

        Arguments:
            order - Order of the butterworth filter
            gyro_cutoff - Cutoff frequency for the gyro data
            acc_cutoff - Cutoff frequency for the accelerometer data
            nsamp - Number of samples to average for the running mean
        """

        if method == 'butter':
            if gyro_cutoff is not None:
                islofreq = False
                if len(gyro_cutoff) == 1:
                    gyro_hi = gyro_cutoff[0]
                elif len(gyro_cutoff) == 2 and (gyro_cutoff[0] == 0):
                    gyro_hi = gyro_cutoff[1]
                elif len(gyro_cutoff) == 2:
                    gyro_hi = gyro_cutoff[1]
                    gyro_lo = gyro_cutoff[0]
                    islofreq = True
                else:
                    raise ValueError('Unrecognized frequency range {}'.format(acc_cutoff))

                # use the SOS form, because it tends to avoid numerical problems for low frequencies
                gyro_sos = signal.butter(order, gyro_hi/(self.sampfreq/2.0), "lowpass", output='sos')
                gyro = signal.sosfiltfilt(gyro_sos, self.gyro0, axis=0)

                if islofreq:
                    gyro -= self.get_low_baseline(self.t0, gyro, gyro_lo)

                self.gyro = gyro
            else:
                self.gyro = self.gyro0

            if acc_cutoff is not None:
                islofreq = False
                if len(acc_cutoff) == 1:
                    acc_hi = acc_cutoff[0]
                    acc = self.acc0
                elif len(acc_cutoff) == 2 and (acc_cutoff[0] == 0):
                    acc_hi = acc_cutoff[1]
                    acc = self.acc0
                elif len(acc_cutoff) == 2:
                    acc_hi = acc_cutoff[1]
                    acc_lo = acc_cutoff[0]
                    islofreq = True
                    acc = self.acc0
                else:
                    raise ValueError('Unrecognized frequency range {}'.format(acc_cutoff))

                acc_sos = signal.butter(order, acc_hi / (self.sampfreq / 2.0), 'lowpass', output='sos')
                acc = signal.sosfiltfilt(acc_sos, acc, axis=0)

                if islofreq:
                    acc -= self.get_low_baseline(self.t0, self.acc0, acc_lo)

                self.acc = acc
            else:
                self.acc = self.acc0

        elif method == 'running':
            self.gyro = np.zeros_like(self.gyro0)
            self.acc = np.zeros_like(self.acc0)

            for i in range(3):
                self.gyro[:, i] = np.convolve(self.gyro0[:, i], np.ones((nsamp,))/nsamp, mode='same')
                self.acc[:, i] = np.convolve(self.acc0[:, i], np.ones((nsamp,))/nsamp, mode='same')

    def get_low_baseline(self, t, y, cutoff, padmode='mean', stat_length=None):
        dur = 1.0 / cutoff
        dt = t[1] - t[0]
        n = int(dur // dt)+1
        N = len(t)

        nblocks = int(np.ceil(float(N)/n))
        print("dt={}, dur={}, N={}, n={}, nblocks={}".format(dt, dur,N,n,nblocks))

        pad = n*nblocks - N

        pad1 = int(pad // 2)
        pad2 = int(pad - pad1)

        y = np.pad(y, ((pad1, pad2), (0, 0)), mode=padmode, stat_length=stat_length)

        yblock = np.split(y, nblocks, axis=0)

        ymn = np.mean(yblock, axis=1)
        ymn = np.pad(ymn, ((1, 1), (0, 0)), mode='edge')
        ctrind = np.arange(nblocks)*n + n/2
        ctrind = np.concatenate(([0], ctrind, [N-1]))
        ind = np.arange(N)

        print("nblocks={}, ymn.shape={}".format(nblocks, ymn.shape))
        ylo = interpolate.interp1d(ctrind, ymn, kind='cubic', axis=0)(ind)
        return ylo

    def _filter_gyro(self, t, gyro):
        if len(self.freqrange) == 1:
            btype = 'lowpass'
        elif len(self.freqrange) == 2:
            btype = 'bandpass'
        else:
            raise ValueError('Unrecognized frequency range {}'.format(self.freqrange))

        sos = signal.butter(self.filterorder, self.freqrange/(self.sampfreq/2.0), btype, output='sos')

        gyros = signal.sosfiltfilt(sos, gyro, axis=0)
        return gyros

    def _resample_gyro(self, t0, gyro0, resamplefreq):
        t = np.arange(t0[0], t0[-1], 1.0/resamplefreq)

        intp = interpolate.interp1d(t0, gyro0, axis=0)
        gyro = intp(t)

        return t, gyro

    def get_orientation(self, method='madgwick', initwindow=0.5, beta=0.05,
                        gain=0.5, gainrange=(0.1, 0.2), epsilon=0.9):
        """Get orientation and dynamic acceleration from IMU data.

        Estimates the orientation as roll, pitch, yaw and the dynamic acceleration.

        Three algorithms are implemented: 'madgwick', 'integrate_gyro', 'dsf', 'valenti'.

        Arguments:
            method - Algorithm ('madgwick', 'integrate_gyro', or 'dsf')
            initwindow - Initial time window to average to get the initial orientation.
            beta - Madgwick beta parameter. beta=0 is equivalent to integrating the gyros

            Valenti algorithm:
            gain - Gain parameter, between 0 and 1.  gain=0 is gyro integration; gain=1 is assuming the
                accelerometer always reads gravity exactly (ie, no dynamic acceleration)
            gainrange - Adaptive gain range. Default = (0.1, 0.2).  Gain = gain below gainrange[0]; gain = 0 above
                gainrange[1], and linear interpolation between the two
            epsilon - Accelerometer interpolation parameter. Default = 0.9

        NB: Internal time, accelerometer, and gyro data should be in sec, m/s^2, and rad/s.
        """

        if method.lower() == 'dsf':
            dt = np.mean(np.diff(self.t))
            self._get_orientation_dsf()

            orient_world = []
            accdyn_world = []
            rotm_world = []
            qorient_world = []
            for chiprpy, adyn1 in zip(self.orient_sensor, self.accdyn_sensor):
                Rchip = self._eul2rotm(chiprpy)
                R = Rchip.dot(self.chip2world_rot)

                worldrotm = self.chip2world_rot.T.dot(R)
                orient_world.append(self._rotm2eul(worldrotm))
                qorient_world.append(self._rotm2quat(worldrotm))
                rotm_world.append(worldrotm)

                # rotate the dynamic acceleration into the world coordinates
                accdyn_world.append(R.T.dot(adyn1))

            self.orient_world = np.array(orient_world)
            self.orient = self.orient_world
            self.orient_world_rotm = np.array(rotm_world)
            self.accdyn_world = np.array(accdyn_world)
            self.accdyn = self.accdyn_world

        elif method.lower() == 'valenti':
            self._get_orientation_valenti(initwindow=initwindow, gain=gain, gainrange=gainrange, epsilon=epsilon)

            qorient_world = [self.qchip2world.conj() * q1.conj() for q1 in self.qorient]
            self.qorient_world = qorient_world

            self.orient_world_rotm = np.array([quaternion.as_rotation_matrix(q1) for q1 in qorient_world])
            self.orient_world = np.array([self._rotm2eul(R1) for R1 in self.orient_world_rotm])
            self.orient = self.orient_world

            # rotate accdyn into the world coordinate system
            qaccdyn_world = [self.qchip2world.conj() * np.quaternion(0, *a1) * self.qchip2world
                             for a1 in self.accdyn_sensor]
            self.accdyn_world = np.array([q.components[1:] for q in qaccdyn_world])
            self.accdyn = self.accdyn_world

        elif method.lower() in ['madgwick', 'integrate_gyro']:
            if method.lower() == 'integrate_gyro':
                beta = 0.0

            self._get_orientation_madgwick(initwindow=initwindow, beta=beta)

            # attempt to convert from chip coordinates to world
            # self.qorient is the quaternion that specifies the current orientation of the chip, relative to its initial
            # orientation, and self.qchip2world is the quaternion that rotates from the initial chip orientation to the
            # world frame
            qorient_world = [self.qchip2world.conj() * q1.conj() for q1 in self.qorient]
            self.qorient_world = np.array(qorient_world)

            self.orient_world_rotm = np.array([quaternion.as_rotation_matrix(q1) for q1 in qorient_world])
            self.orient_world = np.array([self._rotm2eul(R1) for R1 in self.orient_world_rotm])
            self.orient = self.orient_world

            # rotate accdyn into the world coordinate system
            qaccdyn_world = [self.qchip2world.conj() * np.quaternion(0, *a1) * self.qchip2world
                             for a1 in self.accdyn_sensor]
            self.accdyn_world = np.array([q.components[1:] for q in qaccdyn_world])
            self.accdyn = self.accdyn_world

        return self.orient_world

    def _convert_units(self, t=None, acc=None, gyro=None, t_units='ms', acc_units='m/s^2', gyro_units='rad/s'):
        ret = []
        if t is not None:
            if t_units == 'ms' or t_units == 'msec':
                t /= 1000.0
            elif t_units == 's' or t_units == 'sec':
                pass
            else:
                raise ValueError('Unrecognized unit for time: {}'.format(t_units))
            ret.append(t)

        if acc is not None:
            if acc_units.lower() in ['g']:
                acc *= 9.81
            elif acc_units.lower() in ['m/s^2', 'ms2', 'm/sec^2']:
                pass
            else:
                raise ValueError('Unrecognized unit for acceleration: {}'.format(acc_units))
            ret.append(acc)

        if gyro is not None:
            if gyro_units.lower() in ['deg/s', 'deg/sec', 'degs']:
                gyro *= np.pi/180.0
            elif gyro_units.lower() in ['rad/s', 'rad/sec', 'rads']:
                pass
            else:
                raise ValueError('Unrecognized unit for gyroscope: {}'.format(gyro_units))
            ret.append(gyro)

        if len(ret) == 1:
            return ret[0]
        else:
            return tuple(ret)

    def calibrate(self, filename, acc_units='m/s^2', gyro_units='rad/s'):
        """Get initial noise covariances, based on a static recording"""
        with h5py.File(filename, 'r') as h5calib:
            gyro = np.array(h5calib['/data/Gyro'])
            accel = np.array(h5calib['/data/Accel'])

        accel, gyro = self._convert_units(acc=accel, gyro=gyro, acc_units=acc_units, gyro_units=gyro_units)

        self._calibrate(accel, gyro)

    def _calibrate(self, accel, gyro, accdynmag=200.0):
        """Get initial covariances.
        gyro in rad/sec
        accel in m/s^2"""
        self.bias_gyro = np.mean(gyro, axis=0)

        # get noise covariances
        self.Qgyro = np.cov(gyro, rowvar=False)
        self.Qacc = np.cov(accel, rowvar=False)

        # bias noise covariance (assuming low drift)
        self.Qbias = 1e-10 * self.Qacc

        # dynamic acceleration covariance
        # this may be wrong
        self.Qdyn = accdynmag * self._stack_matrices([[self.Qacc, np.zeros((3,3))],
                                                      [np.zeros((3,3)), self.Qacc]])

        # gyro noise
        self.gyro_noise = np.std(gyro, axis=0)

    def estimate_beta(self, filename, gyro_units='rad/s'):
        """Get initial noise covariances, based on a static recording"""
        with h5py.File(filename, 'r') as h5calib:
            gyro = np.array(h5calib['/data/Gyro'])

        gyro = self._convert_units(gyro=gyro, gyro_units=gyro_units)
        beta = np.sqrt(3.0/4.0) * np.max(np.std(gyro, axis=0))

        return beta

    def get_inertial_coords(self, filename, method='mean accel', g=None, acc_units='m/s^2', gyro_units='rad/s'):
        """Get the initial gravity vector"""
        with h5py.File(filename, 'r') as h5inertial:
            accel = np.array(h5inertial['/data/Accel'])

        accel = self._convert_units(acc=accel, acc_units=acc_units)

        if g is not None:
            self.gN = g
        elif method == 'mean accel':
            self.gN = np.mean(accel, axis=0)

    def get_world_coordinates(self, filename=None, axes=('z'), times=None, averagedur=0.1, acc=None, t=None,
                              t_units='ms', acc_units='m/s^2', gyro_units='rad/s'):
        """Get the world coordinates.

        Uses a file where the chip is oriented so that gravity points along what we want as the world axes,
        one at a time.  axes specifies the order.  If we only have one axis (say we only recorded the static g vector),
        we attempt to align the other axes as close to the original chip axes as possible
        """
        axinddict = {'x': 0, 'y': 1, 'z': 2}

        if times is not None:
            times = np.array(times)

        if acc is None:
            if isinstance(filename, str):
                with h5py.File(filename, 'r') as h5calib:
                    acc = np.array(h5calib['/data/Accel'])
                    t = np.array(h5calib['/data/t']).astype(float)
                t, acc = self._convert_units(t=t, acc=acc, t_units=t_units, acc_units=acc_units)
            else:
                tmax = 0
                t = np.array([])
                acc = np.zeros((0, 3))
                for i, fn in enumerate(filename):
                    with h5py.File(fn, 'r') as h5calib:
                        acc1 = np.array(h5calib['/data/Accel'])
                        t1 = np.array(h5calib['/data/t']).astype(float)

                    t1, acc1 = self._convert_units(t=t1, acc=acc1, t_units=t_units, acc_units=acc_units)

                    acc = np.append(acc, acc1, axis=0)
                    t = np.append(t, t1+tmax)
                    times[i] += tmax
                    tmax += np.max(t1)
        gax = np.eye(3)

        if len(axes) == 1:
            times = [0]
            averagedur = 4*t[-1]

        d2 = averagedur/2

        axord = []
        for axis1, time1 in zip(axes, times):
            ist = np.logical_and(t >= time1-d2, t <= time1+d2)
            ax = np.mean(acc[ist, :], axis=0)

            m = re.match('([+-]?)([XYZxyz])', axis1)
            if m is None:
                raise ValueError('Unrecognized axis specification axis')

            if m.group(1) == '-':
                axsign = -1.0
            else:
                axsign = 1.0

            axind = axinddict[m.group(2).lower()]

            gax[:, axind] = ax * axsign
            axord.append(axind)

        axord = np.concatenate((axord, np.setdiff1d(range(3), axord)))
        axrevord = np.argsort(np.arange(3)[axord])

        basis = self._gramschmidt(gax[:, axord])
        basis = basis[:, axrevord]

        if len(axes) == 3:
            # check for right handed-ness
            # the Z axis should be equal to the cross product of the X and Y axes
            # because of small numerical issues, it's sometimes not exactly equal,
            # so we check that they're in the same direction
            assert(np.dot(np.cross(basis[:, 0], basis[:, 1]), basis[:, 2]) > 0.9)
        else:
            if np.dot(np.cross(basis[:, 0], basis[:, 1]), basis[:, 2]) < -0.5:
                basis[:, 2] = -basis[:, 2]

        self.chip2world_rot = basis
        self.axord_world = axord
        self.qchip2world = quaternion.from_rotation_matrix(basis)
        self.qworld2chip = quaternion.from_rotation_matrix(basis.T)

    def _gramschmidt(self, U):
        k = U.shape[1]
        V = copy(U)

        for i in range(k):
            V[:, i] /= np.linalg.norm(V[:, i])

            for j in range(i+1, k):
                proj = np.dot(V[:, i], V[:, j]) * V[:, i]
                V[:, j] -= proj

        return V

    def _get_orientation_dsf(self):
        """Dynamic snap free Kalman filter for sensor fusion
        x is the state: [theta, bias, adyn]^T
        where theta are the Euler """
        if self.Qbias is None or self.Qgyro is None or self.Qacc is None or self.Qdyn is None:
            raise ValueError('Noise covariance is not yet estimated')
        if self.gN is None:
            raise ValueError('Inertial reference frame is not yet set')

        xkm1 = np.zeros((12,))
        dt = np.diff(self.t)
        dt = np.insert(dt, 0, dt[0:0])
        dt1 = dt[0]

        # make 9 x 9 matrix
        Pkm1 = self._stack_matrices([
            [(self.Qgyro + self.Qbias)*dt1**2,  -self.Qbias*dt1,    np.zeros((3, 6))],
            [-self.Qbias*dt1,                   self.Qbias,         np.zeros((3, 6))],
            [np.zeros((6, 3)),                  np.zeros((6, 3)),   self.Qdyn]])

        N = self.gyro.shape[0]

        gyro = self.gyro - self.bias_gyro
        acc = self.acc

        eulerEKF = []
        aD = []
        err = []
        PkEKF = []
        xkEKF = []
        Rk = self.Qacc

        for dt1, omegak, accel in zip(dt, gyro, acc):
            Fk, xkM, Bk = self._system_dynamics(xkm1, omegak, dt1)

            Qk = self._stack_matrices([
                [Bk.dot(self.Qgyro + self.Qbias).dot(Bk.T)*dt1**2,    -Bk.dot(self.Qbias)*dt1,  np.zeros((3, 6))],
                [-self.Qbias.dot(Bk.T)*dt1,                           self.Qbias,               np.zeros((3, 6))],
                [np.zeros((6, 6)),                                                              self.Qdyn]])

            PkM = Fk.dot(Pkm1).dot(Fk.T) + Qk
            hk, Jh = self._observation_dynamics(xkM, self.gN)

            Hk = Jh

            Sk = Hk.dot(PkM).dot(Hk.T) + Rk
            Kk = PkM.dot(Hk.T).dot(np.linalg.pinv(Sk))
            xk = xkM + Kk.dot(accel - hk)
            Pk = (np.eye(12) - Kk.dot(Hk)).dot(PkM)

            QT = self._eul2rotm(xk[:3])

            eulerEKF.append(xk[:3])
            aD.append(QT.T.dot(xk[6:9]))
            err.append(accel - ((QT.dot(self.gN) + xk[6:9])))
            PkEKF.append(Pk)
            xkEKF.append(xk)

            Pkm1 = Pk
            xkm1 = xk

        xkEKF = np.array(xkEKF)

        self.orient_sensor = np.pad(np.array(eulerEKF), ((1, 0), (0, 0)), mode='edge')
        self.accdyn_sensor = np.pad(np.array(aD), ((1, 0), (0, 0)), mode='edge')
        self.jerk_sensor = np.pad(xkEKF[:, 6:9], ((1, 0), (0, 0)), mode='edge')
        self.snap_sensor = np.pad(xkEKF[:, 9:], ((1, 0), (0, 0)), mode='edge')

        qorient = []
        for o1 in self.orient_sensor:
            qorient.append(quaternion.from_euler_angles(*o1))
        self.qorient = np.array(qorient)

    def _system_dynamics(self, xk, omegak, dt):
        phi, theta, psi = xk[:3]
        biask = xk[3:6]

        sPh = np.sin(phi)
        cPh = np.cos(phi)
        tTh = np.tan(theta)
        scTh = 1 / np.cos(theta)

        Bk = np.array([[1,  sPh*tTh,    cPh*tTh],
                       [0,  cPh,        -sPh],
                       [0,  sPh*scTh,   cPh*scTh]])

        # partial diffs
        Bk_phi = np.array([[0,  cPh*tTh,    -sPh*tTh],
                           [0,  -sPh,       -cPh],
                           [0,  cPh*scTh,   -sPh*scTh]])

        Bk_theta = np.array([[0,    sPh*scTh**2,    cPh*scTh**2],
                             [0,    0,              0],
                             [0,    sPh*scTh*tTh,   cPh*scTh*tTh]])

        Bk_psi = np.zeros((3, 3))

        unbiased_omegak = omegak - biask
        unbiased_omegak = unbiased_omegak[:, np.newaxis]

        Bkomega = np.hstack((np.dot(Bk_phi, unbiased_omegak),
                             np.dot(Bk_theta, unbiased_omegak),
                             np.dot(Bk_psi, unbiased_omegak)))

        jerk = xk[9:]

        Fk = self._stack_matrices([
            [np.eye(3) + Bkomega*dt, -Bk*dt, np.zeros((3, 6))],
            [np.zeros((3, 3)), np.eye(3), np.zeros((3, 6))],
            [np.zeros((3, 6)), np.eye(3), np.eye(3)*dt],
            [np.zeros((3, 6)), np.zeros((3, 3)), np.eye(3)]])

        xkp1 = np.hstack((xk[:3] + np.dot(Bk, unbiased_omegak).squeeze()*dt, xk[3:6],
                          xk[6:9] + jerk*dt, xk[9:]))

        return Fk, xkp1, Bk

    def _observation_dynamics(self, xk, gN):
        """gN = gravity in inertial coordinate system (3x1)"""
        phi, theta, psi = xk[:3]

        # rotation matrices
        Rz_yaw =    np.array([[np.cos(psi),     np.sin(psi),    0],
                             [-np.sin(psi),     np.cos(psi),    0],
                             [0,                0,              1]])
        Ry_pitch = np.array([[np.cos(theta),    0,              -np.sin(theta)],
                             [0,                1,              0],
                             [np.sin(theta),    0,              np.cos(theta)]])
        Rx_roll =  np.array([[1,                0,              0],
                             [0,                np.cos(phi),    np.sin(phi)],
                             [0,                -np.sin(phi),   np.cos(phi)]])

        # rates/derivatives
        Rz_yaw_rate =   np.array([[-np.sin(psi),    np.cos(psi),    0],
                                  [-np.cos(psi),    -np.sin(psi),   0],
                                  [0,               0,              0]])
        Ry_pitch_rate = np.array([[-np.sin(theta),  0,              -np.cos(theta)],
                                  [0,               0,              0],
                                  [np.cos(theta),   0,              -np.sin(theta)]])
        Rx_roll_rate =  np.array([[0,               0,              0],
                                  [0,               -np.sin(phi),   np.cos(phi)],
                                  [0,               -np.cos(phi),   -np.sin(phi)]])

        QT       = Rx_roll.dot(Ry_pitch).dot(Rz_yaw)
        QT_roll  = Rx_roll_rate.dot(Ry_pitch).dot(Rz_yaw)
        QT_pitch = Rx_roll.dot(Ry_pitch_rate).dot(Rz_yaw)
        QT_yaw   = Rx_roll.dot(Ry_pitch).dot(Rz_yaw_rate)

        Jh = np.vstack((QT_roll.dot(gN), QT_pitch.dot(gN), QT_yaw.dot(gN), np.zeros((3, 3)),
                        np.eye(3), np.zeros((3, 3)))).T
        hk = QT.dot(gN) + xk[6:9]

        return hk, Jh

    def _eul2rotm(self, x):
        phi, theta, psi = x

        Rz_yaw =    np.array([[np.cos(psi),     np.sin(psi),    0],
                             [-np.sin(psi),     np.cos(psi),    0],
                             [0,                0,              1]])
        Ry_pitch = np.array([[np.cos(theta),    0,              -np.sin(theta)],
                             [0,                1,              0],
                             [np.sin(theta),    0,              np.cos(theta)]])
        Rx_roll =  np.array([[1,                0,              0],
                             [0,                np.cos(phi),    np.sin(phi)],
                             [0,                -np.sin(phi),   np.cos(phi)]])
        QT = Rx_roll.dot(Ry_pitch.dot(Rz_yaw))

        return QT

    def _rotm2eul(self, rotm, singularity=0.001):
        # if rotm[0, 2] < -1 + singularity:
        #     psi = 0
        #     theta = -np.pi/2
        #     phi = np.arctan2(-rotm[1, 0], -rotm[2, 0])
        # elif rotm[2, 0] > 1 - singularity:
        #     psi = 0
        #     theta = np.pi / 2
        #     phi = np.arctan2(rotm[1, 0], rotm[2, 0])
        # else:
        phi = np.arctan2(rotm[1, 2], rotm[2, 2])
        theta = np.arcsin(rotm[0, 2])
        psi = np.arctan2(rotm[0, 1], rotm[0, 0])

        return phi, theta, psi

    def _rotm2quat(self, R):
        if R[1, 1] > -R[2, 2] and R[0, 0] > -R[1, 1] and R[0, 0] > -R[2, 2]:
            a = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2])
            q = 0.5 * np.quaternion(a, (R[1, 2] - R[2, 1]) / a, (R[2, 0] - R[0, 2]) / a, (R[0, 1] - R[1, 0]) / a)
        elif R[1, 1] < -R[2, 2] and R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            a = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
            q = 0.5 * np.quaternion((R[1, 2] - R[2, 1]) / a, a, (R[0, 1] + R[1, 0]) / a, (R[2, 0] + R[0, 2]) / a)
        elif R[1, 1] > R[2, 2] and R[0, 0] < R[1, 1] and R[0, 0] < -R[2, 2]:
            a = np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2])
            q = 0.5 * np.quaternion((R[2, 0] - R[0, 2]) / a, (R[0, 1] + R[1, 0]) / a, a, (R[1, 2] + R[2, 1]) / a)
        elif R[1, 1] < R[2, 2] and R[0, 0] < -R[1, 1] and R[0, 0] < R[2, 2]:
            a = np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2])
            q = 0.5 * np.quaternion((R[0, 1] - R[1, 0]) / a, (R[2, 0] + R[0, 2]) / a, (R[1, 2] + R[2, 1]) / a, a)

        return q

    def _eul2quat(self, x):
        phi, theta, psi = x

        q_yaw = np.quaternion(np.cos(0.5*psi), 0, 0, np.sin(0.5*psi))
        q_pitch = np.quaternion(np.cos(0.5*theta), 0, np.sin(0.5*theta), 0)
        q_roll = np.quaternion(np.cos(0.5*phi), np.sin(0.5*phi), 0, 0)

        return q_roll * q_pitch * q_yaw

    def _correct_singularity(self, rpy, tol=0.01):
        jump = np.pi - tol

        rpycorr = copy(rpy)
        for rpy1, rpynext in zip(rpycorr, rpycorr[1:, :]):
            if rpynext[0] - rpy1[0] >= jump:
                rpynext[0] -= np.pi
                if rpynext[2] >= jump:
                    rpynext[1] = -rpynext[1] - np.pi
                    rpynext[2] -= np.pi
                else:
                    rpynext[2] += np.pi
            elif rpynext[0] - rpy1[0] <= -jump:
                rpynext[0] += np.pi
                rpynext[1] = -rpynext[1] - np.pi
                rpynext[2] -= np.pi

        return rpycorr

    def _stack_matrices(self, M):
        m = []
        for row in M:
            m.append(np.hstack(tuple(row)))
        return np.vstack(tuple(m))

    def _get_orientation_madgwick(self, initwindow=0.5, beta=0.05):
        """beta in rad"""
        gyrorad = self.gyro
        betarad = beta

        qorient = np.zeros_like(self.t, dtype=np.quaternion)
        qgyro = np.zeros_like(self.t, dtype=np.quaternion)

        dt = 1.0 / self.sampfreq

        isfirst = self.t <= self.t[0] + initwindow
        # qorient[0] = self.orientation_from_accel(np.mean(self.acc[isfirst, :], axis=0))
        qorient[0] = self.qchip2world.conj()

        for i, gyro1 in enumerate(gyrorad[1:, :], start=1):
            qprev = qorient[i-1]

            # acc is turned into a unit vector, so it doesn't matter what units it's in
            acc1 = self.acc[i, :]
            acc1 = acc1 / np.linalg.norm(acc1)

            # quaternion angular change from the gryo
            qdotgyro = 0.5 * (qprev * np.quaternion(0, *gyro1))
            qgyro[i] = qprev + qdotgyro * dt

            if beta > 0:
                # gradient descent algorithm corrective step
                qp = qprev.components
                F = np.array([2*(qp[1]*qp[3] - qp[0]*qp[2]) - acc1[0],
                              2*(qp[0]*qp[1] + qp[2]*qp[3]) - acc1[1],
                              2*(0.5 - qp[1]**2 - qp[2]**2) - acc1[2]])
                J = np.array([[-2*qp[2], 2*qp[3], -2*qp[0], 2*qp[1]],
                               [2*qp[1], 2*qp[0], 2*qp[3], 2*qp[2]],
                               [0, -4*qp[1], -4*qp[2], 0]])

                step = np.dot(J.T, F)
                step = step / np.linalg.norm(step)
                step = np.quaternion(*step)

                qdot = qdotgyro - betarad * step
            else:
                qdot = qdotgyro

            qorient[i] = qprev + qdot * dt
            qorient[i] /= np.abs(qorient[i])

        # get the gravity vector
        # gravity is +Z
        gvec = np.array([(q.conj() * np.quaternion(0, 0, 0, 1) * q).components[1:] for q in qorient])

        self.qorient = qorient
        self.orient_sensor = np.array([self._rotm2eul(quaternion.as_rotation_matrix(self.qchip2world * q1))
                                       for q1 in qorient])
        self.gvec = 9.81 * gvec
        self.accdyn_sensor = self.acc - gvec

    def _get_orientation_valenti(self, initwindow=0.5, gain=0.5, gainrange=(0.1, 0.2), epsilon=0.9):
        def adapt_gain(gain, err, gainrange=(0.1, 0.2)):
            if err < gainrange[0]:
                return gain
            elif err < gainrange[1]:
                return gain * (1.0 - (err - gainrange[0]) / (gainrange[1] - gainrange[0]))
            else:
                return 0.0

        gyrorad = np.deg2rad(self.gyro)

        qorientGL = np.zeros_like(self.t, dtype=np.quaternion)

        qgyro = np.zeros_like(self.t, dtype=np.quaternion)
        gvec = np.zeros_like(self.gyro)

        dt = 1.0 / self.sampfreq

        isfirst = self.t <= self.t[0] + initwindow
        # qorientGL[0] = self.orientation_from_accel(np.mean(self.acc[isfirst, :], axis=0)).conj()
        qorientGL[0] = self.qchip2world

        if gainrange is None:
            gainfcn = lambda gain, e, r: gain
        elif len(gainrange) == 2:
            gainfcn = adapt_gain

        self.err = [0]
        self.alpha = [gain]

        for i, (gyro1, acc1) in enumerate(zip(gyrorad[1:, :], self.acc[1:, :]), start=1):
            qprev = qorientGL[i-1]

            # first integrate gyro
            qdotgyroGL = -0.5 * np.quaternion(0, *gyro1) * qprev
            qgyro1 = qprev + qdotgyroGL * dt

            err = np.abs(np.linalg.norm(acc1) - 1.0)
            alpha = gainfcn(gain, err, gainrange)

            self.err.append(err)
            self.alpha.append(alpha)

            # correction relative to gravity

            # use the integrated gyro to estimate the gravity vector
            acc1 = acc1 / np.linalg.norm(acc1)

            grav_est = qgyro1.conj() * np.quaternion(0, *acc1) * qgyro1
            grav_est = grav_est.components[1:]

            # this is the quaternion rotation to bring our estimated gravity in line with the true one ([0, 0, 1])
            deltaqacc = np.quaternion(np.sqrt((grav_est[2]+1)/2), -grav_est[1]/np.sqrt(2*(grav_est[2]+1)),
                                      grav_est[0]/np.sqrt(2*(grav_est[2]+1)), 0)

            # interpolate between qI = [1 0 0 0] (= no rotation) and deltaqacc
            qI = np.quaternion(1, 0, 0, 0)
            if deltaqacc.components[0] > epsilon:
                # use linear interpolation
                deltaqacc = (1 - alpha)*qI + alpha*deltaqacc
                deltaqacc = deltaqacc / abs(deltaqacc)
            else:
                # use spherical linear interpolation

                # dot product of quaternions is cos of angle between them
                # only non-zero element of qI is the first one
                Omega = np.arccos(qI.components[0] * deltaqacc.components[0])

                deltaqacc = np.sin((1 - alpha)*Omega)/np.sin(Omega) * qI + \
                            np.sin(alpha * Omega)/np.sin(Omega) * deltaqacc

            qorientGL[i] = qgyro1 * deltaqacc

        self.err = np.array(self.err)
        self.alpha = np.array(self.alpha)

        # qorientGL is the quaternion for the global frame relative to the local.  We want it the other way round
        qorient = np.array([q1.conj() for q1 in qorientGL])

        # get the gravity vector
        # gravity is +Z
        gvec = [(q.conj() * np.quaternion(0, 0, 0, 1) * q).components[1:] for q in qorient]

        self.qorient = qorient
        self.orient_sensor = np.array([self._rotm2eul(quaternion.as_rotation_matrix(self.qchip2world * q1))
                                       for q1 in qorient])
        self.gvec = 9.81 * gvec
        self.accdyn_sensor = self.acc - gvec

    def integrate_gyro(self, initwindow=0.5):
        gyrorad = self.gyro

        qorient = np.zeros_like(self.t, dtype=np.quaternion)
        gvec = np.zeros_like(self.gyro)

        dt = 1.0 / self.sampfreq

        isfirst = self.t <= self.t[0] + initwindow
        qorient[0] = self.orientation_from_accel(np.mean(self.acc[isfirst, :], axis=0))

        for i, gyro1 in enumerate(gyrorad[1:, :], start=1):
            qprev = qorient[i-1]

            # quaternion angular change from the gyro
            qdotgyro = 0.5 * (qprev * np.quaternion(0, *gyro1))
            qorient[i] = qprev + qdotgyro * dt

            g1 = qorient[i].conj() * np.quaternion(0, 0, 0, 1) * qorient[i]
            gvec[i, :] = g1.components[1:]

        self.qorient = qorient
        self.orient_sensor = quaternion.as_euler_angles(qorient)
        self.gvec = 9.81 * gvec
        self.accdyn_sensor = self.acc - gvec

    def orientation_from_accel(self, acc):
        """Get a quaternion orientation from an accelerometer reading, assuming that the accelerometer correctly
        measures the gravitational acceleration.
        
        Equation from Valenti, Dryanovski, and Xiao 2015 (Sensors 15)"""
        acc1 = acc / np.linalg.norm(acc)

        ax, ay, az = acc1

        if az >= 0:
            q0 = np.quaternion(np.sqrt((az+1)/2), ay/np.sqrt(2*(az+1)), -ax/np.sqrt(2*(az+1)), 0)
        else:
            q0 = np.quaternion(-ay/np.sqrt(2*(1-az)), -np.sqrt((1-az)/2), 0, -ax/np.sqrt(2*(1-az)))

        # Other equation - not sure where from...
        # AXZ = ax * np.sqrt(1 - az)
        # AXY = np.sqrt(ax**2 + ay**2)
        # q0 = np.quaternion(0, AXZ / (np.sqrt(2)*AXY), ay*AXZ / (np.sqrt(2) * ax * AXY), ax*AXY / (np.sqrt(2)*AXZ))
        q0 = q0 / np.abs(q0)

        return q0



def main():
    # run tests to see if the algorithms behave nicely

    filename = 'two_imu_data/data_b_2.hdf5'
    calibfilename = 'two_imu_data/data_b_1.hdf5'
    # encoderfilename = '/Users/etytel01/Documents/Acceleration/AlgoComparisons/Planar Experiment/encoder_8.dat'

    plt.ion()

    imu = IMU()

    imu.calibrate(calibfilename)
    imu.get_inertial_coords(calibfilename)
    imu.get_world_coordinates(calibfilename)

    imu.load(filename, resamplefreq=200.0)

    imu.filter(nsamp=10, method='running')

    with h5py.File(filename, 'r') as h5file:
        enc0 = np.array(h5file['/data/Encoder'])
        t0 = np.array(h5file['/data/t'])
    t0 /= 1000.0

    t = imu.t
    enc = interpolate.interp1d(t0, enc0)(t)

    # fig, ax = plt.subplots()
    # ax.plot(t, enc)

    imu.get_orientation(method='dsf')
    orient_dsf = copy(imu.orient_world)
    accd1 = copy(imu.accdyn)

    imu.get_orientation(method='madgwick')
    orient_mad = copy(imu.orient_world)
    imu.get_orientation(method='integrate_gyro')
    orient_gyro = copy(imu.orient_world)

    enc -= enc[0]
    enc = -enc

    fig, ax = plt.subplots()
    ax.plot(t, enc, label='encoder')
    ax.plot(imu.t, np.rad2deg(orient_dsf[:, 0]), label='dsf')
    ax.plot(imu.t, np.rad2deg(orient_mad[:, 0]), label='madgwick')
    ax.plot(imu.t, np.rad2deg(orient_gyro[:, 0]), label='gyro')
    ax.legend()

    fig, ax = plt.subplots()
    ax.plot(imu.t, np.rad2deg(orient_dsf[:, 0]) - enc, label='dsf')
    ax.plot(imu.t, np.rad2deg(orient_mad[:, 0]) - enc, label='madgwick')
    ax.plot(imu.t, np.rad2deg(orient_gyro[:, 0]) - enc, label='gyro')
    ax.set_title('Error')
    ax.legend()

    plt.show(block=True)

if __name__ == "__main__":
    main()
