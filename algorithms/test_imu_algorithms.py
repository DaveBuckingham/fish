import numpy as np
from scipy import interpolate, signal
import quaternion
import h5py
import logging
from copy import copy

import matplotlib.pyplot as plt

from process_imu import IMU

class IMU_test_data(IMU):
    def __init__(self, trange):
        self.t = np.arange(trange[0], trange[1], trange[2])
        self.t0 = self.t
        self.sampfreq = 1.0/trange[2]

    def set_initial_orientation(self, R0):
        '''Set the initial orientation of the chip.  
        
        R0 is the rotation matrix from world to chip, so that xchip = R0.dot(xworld) takes a vector xworld in
        world coordinates and converts it to chip coordinates'''
        self.R0 = R0
        self.q0 = quaternion.from_rotation_matrix(R0)

    def set_noise_params(self, gyrorms, gyrofreq, gyrodriftrms, gyrodriftfreq, accrms, accfreq):
        self.gyrorms = gyrorms
        self.gyrofreq = gyrofreq
        self.gyrodriftfreq = gyrodriftfreq
        self.gyrodriftrms = gyrodriftrms
        self.accrms = accrms
        self.accfreq = accfreq

    def calibrate(self, filename=None, duration=30.0):
        dt = self.t[1] - self.t[0]
        t = np.arange(0, duration, dt)

        g_imu = np.matmul(self.R0, np.array([0, 0, 1.0]))

        acc = np.tile(g_imu[np.newaxis], (len(t), 1))
        gyro = np.zeros((len(self.t), 3))

        accn, gyron = self._add_noise(acc, gyro)

        accn = 9.81 * accn
        gyron = np.deg2rad(gyron)

        self._calibrate(accn, gyron)

    def get_inertial_coords(self, filename=None, method='mean accel', g=None, duration=30.0):
        dt = self.t[1] - self.t[0]
        t = np.arange(0, duration, dt)
        g_imu = np.matmul(self.R0, np.array([0, 0, 1.0]))

        acc = np.tile(g_imu[np.newaxis], (len(t), 1))
        gyro = np.zeros_like(acc)

        accn, gyron = self._add_noise(acc, gyro)

        self.gN = 9.81 * np.mean(accn, axis=0)

    def get_world_coordinates(self, duration=30.0, **kwargs):
        # first oriented with gravity aligned with z axis
        R1 = np.eye(3,3)
        # then rolled so that gravity is aligned with +y
        R2 = self._eul2rotm((np.deg2rad(90), 0, 0))
        # then pitched so that gravity is aligned with +x
        R3 = self._eul2rotm((0, np.deg2rad(-90), 0))

        d1 = duration/3
        dt = self.t[1] - self.t[0]
        t = np.arange(0, duration, dt)

        n1 = int(np.floor(d1/dt))

        Rt = np.vstack((np.tile(R1[np.newaxis, :, :], (n1, 1, 1)),
                       np.tile(R2[np.newaxis, :, :], (n1, 1, 1)),
                       np.tile(R3[np.newaxis, :, :], (n1, 1, 1))))

        R = np.matmul(self.R0, Rt)

        acc = np.matmul(R, np.array([0, 0, 1.0]))
        gyro = np.zeros_like(acc)

        accn, gyron = self._add_noise(acc, gyro)

        super(IMU_test_data, self).get_world_coordinates(acc=accn, t=t, times=[0.5*d1, 1.5*d1, 2.5*d1],
                                                         axes=['z', 'y', 'x'], **kwargs)

    def generate_arm_data(self, alpham, Am, f, delta, l, accaxis=0, rotaxis=1, delay=1.0):
        t = self.t - delay

        # angle of the arm, about the world's rotaxis (default=y) axis
        alpha = alpham * np.sin(2 * np.pi * f * t)
        alphavel = 2 * np.pi * f * alpham * np.cos(2 * np.pi * f * t)
        alphaacc = -4 * np.pi ** 2 * f ** 2 * alpham * np.sin(2 * np.pi * f * t)
        alpha[t <= 0] = 0
        alphavel[t <= 0] = 0
        alphaacc[t <= 0] = 0

        # acceleration of the body in the world frame
        accb = np.zeros((len(t), 3))
        accb[:, accaxis] = -4 * np.pi**2 * f**2 * Am * np.sin(2*np.pi*(f*t - delta))
        accb[t <= 0, accaxis] = 0

        # angular velocities in the world frame
        omega = np.zeros_like(accb)
        omega[:, rotaxis] = alphavel
        omegadot = np.zeros_like(omega)
        omegadot[:, rotaxis] = alphaacc
        self.omega_world = omega

        self.gyro_nonoise = np.array([self.R0.dot(o1) for o1 in omega])

        omegab = self.gyro_nonoise
        omegabdot = np.array([self.R0.dot(od1) for od1 in omegadot])

        # rotation matrix from the world to the body frame
        if rotaxis == 0:
            Rt = np.array([[np.ones_like(t), np.zeros_like(t), np.zeros_like(t)],
                           [np.zeros_like(t), np.cos(alpha), np.sin(alpha)],
                           [np.zeros_like(t), -np.sin(alpha), np.cos(alpha)]])
        elif rotaxis == 1:
            Rt = np.array([[np.cos(alpha), np.zeros_like(t), -np.sin(alpha)],
                           [np.zeros_like(t), np.ones_like(t), np.zeros_like(t)],
                           [np.sin(alpha), np.zeros_like(t), np.cos(alpha)]])
        elif rotaxis == 2:
            Rt = np.array([[np.cos(alpha), np.sin(alpha), np.zeros_like(t)],
                           [-np.sin(alpha), np.cos(alpha), np.zeros_like(t)],
                           [np.zeros_like(t), np.zeros_like(t), np.ones_like(t)]])

        Rt = np.rollaxis(Rt, 2)

        # R is the rotation vector that gets from the world to the current chip orientation
        R = np.matmul(self.R0, Rt)
        self.R = R

        # position of the IMU relative to the body, in the world coordinates
        rvec = np.zeros((3,))
        rvec[rotaxis] = 1.0
        avec = np.zeros((3,))
        avec[accaxis] = 1.0

        armvec = np.cross(avec, rvec)
        xpw = l * armvec
        # convert to body coordinates
        xpb = np.matmul(self.R0, xpw)

        angacc = np.cross(omegabdot, xpb)
        centripacc = np.cross(omegab, np.cross(omegab, xpb))
        adw = accb[:, :, np.newaxis] + np.matmul(np.transpose(R, (0, 2, 1)),
                                                 angacc[:, :, np.newaxis] + centripacc[:, :, np.newaxis])
        adb = np.matmul(R, adw)

        self.accdyn_world_true = np.squeeze(adw)
        self.accdyn_imu_true = np.squeeze(adb)
        self.angacc = angacc
        self.centripacc = centripacc

        uw = []
        qw = []
        uc = []
        for R1 in R:
            # roll, pitch, yaw in world frame
            roll, pitch, yaw = self._rotm2eul(np.matmul(self.R0.T, R1))
            uw.append([roll, pitch, yaw])

            qw.append(np.conj(self._rotm2quat(np.matmul(self.R0.T, R1))))

            # roll, pitch, yaw in chip frame
            roll, pitch, yaw = self._rotm2eul(np.matmul(self.R0, R1.T))
            uc.append([roll, pitch, yaw])

        self.orient_world_true = np.array(uw)
        self.qorient_world_true = np.array(qw)
        self.orient_imu_true = np.array(uc)

        self.orient_world_rotm_true = np.array([np.matmul(self.R0.T, R1) for R1 in R])

        # roll = np.arctan2(R[:, 1, 2], R[:, 2, 2])
        # pitch = -np.arcsin(R[:, 0, 2])
        # yaw = np.arctan2(R[:, 0, 1], R[:, 0, 0])
        # self.orient_world_true = np.vstack((roll, pitch, yaw)).T

        # R1 = R.transpose((0, 2, 1))
        # roll_imu = np.arctan2(R1[:, 1, 2], R1[:, 2, 2])
        # pitch_imu = -np.arcsin(R1[:, 0, 2])
        # yaw_imu = np.arctan2(R1[:, 0, 1], R1[:, 0, 0])
        # self.orient_imu_true = np.vstack((roll_imu, pitch_imu, yaw_imu)).T

        g = np.array([0, 0, 1.0])
        self.g_imu = np.array([np.matmul(R1, g) for R1 in R])

        # convert to gs
        self.accdyn_world_true /= 9.81
        self.accdyn_imu_true /= 9.81
        self.acc_nonoise = np.squeeze(self.accdyn_imu_true + self.g_imu)

        # convert to deg/sec
        self.gyro_nonoise = np.rad2deg(np.squeeze(self.gyro_nonoise))
        self.orient_world_true = np.squeeze(self.orient_world_true)
        self.orient_imu_true = np.squeeze(self.orient_imu_true)

        self.acc0, self.gyro0 = self._add_noise(self.acc_nonoise, self.gyro_nonoise)

    def _add_noise(self, accel, gyro):
        sampfreq = 1.0 / (self.t[1] - self.t[0])

        # accelerometer noise

        n = np.random.standard_normal(accel.shape)
        # no covariance among channels

        sos = signal.butter(9, self.accfreq / (sampfreq / 2), btype='highpass', output='sos')
        acc_noise = signal.sosfilt(sos, n, axis=0)
        acc_noise /= np.sqrt(np.mean(np.square(acc_noise), axis=0))
        acc_noise *= self.accrms
        accn = accel + acc_noise

        # gyro noise

        # first general noise
        n = np.random.standard_normal(gyro.shape)

        sos = signal.butter(9, self.gyrofreq / (sampfreq / 2), btype='highpass', output='sos')
        gyro_noise = signal.sosfilt(sos, n, axis=0)
        gyro_noise /= np.sqrt(np.mean(np.square(gyro_noise), axis=0))
        gyro_noise *= self.gyrorms

        # then drift
        if self.gyrodriftrms > 0.0:
            n = np.random.standard_normal(gyro.shape)

            sos = signal.butter(9, self.gyrodriftfreq / (sampfreq / 2), btype='lowpass', output='sos')
            gyro_drift = signal.sosfilt(sos, n, axis=0)
            gyro_drift /= np.sqrt(np.mean(np.square(gyro_drift), axis=0))
            gyro_drift *= self.gyrodriftrms
        else:
            gyro_drift = np.zeros_like(gyro)

        gyron = gyro + gyro_noise + gyro_drift

        return accn, gyron

def main():
    logging.basicConfig(level=logging.DEBUG)
    # run tests to see if the algorithms behave nicely

    imu = IMU_test_data((0, 20, 1.0/200))

    # initial chip orientation is on its side. x axis up, y axis forward, z axis left. But misaligned slightly
    R0 = imu._eul2rotm(np.deg2rad(np.array([0, 0, 0])))
    imu.set_initial_orientation(R0)

    imu.set_noise_params(gyrorms=5.0,          # deg/sec
                         gyrofreq=20.0,        # Hz
                         gyrodriftrms=0.0,     # deg/sec
                         gyrodriftfreq=0.1,    # Hz
                         accrms=0.03,          # g
                         accfreq=33)           # Hz

    imu.generate_arm_data(alpham=np.deg2rad(15), # 15deg
                              Am=0.0, #0.15, # 15 cm back and forth
                              f=0.7, # Hz oscillation
                              l=0.4,
                              delta=0.2) # 20% phase lag between angle and forward back motion

    fig, ax = plt.subplots(2,1)
    ax[0].plot(imu.t, imu.acc0)
    ax[0].plot(imu.t, imu.acc_nonoise, '--')
    ax[0].plot(imu.t, imu.g_imu, ':')
    ax[0].set_ylabel('Accel')

    ax[1].plot(imu.t, imu.gyro0)
    ax[1].plot(imu.t, imu.gyro_nonoise, '--')
    ax[1].set_ylabel('Gyro')
    ax[1].set_xlabel('Time')

    fig, ax = plt.subplots(3,1)
    for o1, ax1, lab in zip(np.rollaxis(imu.orient_imu_true, 1), ax, ['roll', 'pitch', 'yaw']):
        ax1.plot(imu.t, np.unwrap(o1), label='true')
        ax1.set_ylabel(lab)
    ax[0].set_title('True')

    imu.filter(nsamp=10, method='running')

    imu.calibrate(duration=30.0)
    imu.get_inertial_coords(duration=30.0)
    imu.get_world_coordinates(duration=30.0)

    imu.get_orientation(method='dsf', Ca=(0.0, 0.0, 0.0))
    orient_dsf1 = np.rad2deg(copy(imu.orient_sensor))
    accd1 = copy(imu.accdyn)

    fig, ax = plt.subplots(3,1)
    for o1, ax1, lab in zip(np.rollaxis(orient_dsf1, 1), ax, ['roll', 'pitch', 'yaw']):
        ax1.plot(imu.t, np.unwrap(o1), label='ekf')
        ax1.set_ylabel(lab)
    ax[0].set_title('EKF')

    # imu.get_world_coordinates(duration=30.0)
    #
    # imu.filter(method='butter', order=5, gyro_cutoff=(0.1, 10), acc_cutoff=(0, 30))
    #
    # imu.get_orientation(method='valenti', gain=1.0, gainrange=None)     # should be strictly accelerometer
    # qorient_vala = copy(imu.qorient)
    # orient_vala = np.array([imu._rotm2eul(quaternion.as_rotation_matrix(q1)) for q1 in qorient_vala])
    #
    # imu.get_orientation(method='valenti', gain=0.0, gainrange=None)     # should be strictly gyro
    # qorient_valg = copy(imu.qorient)
    # orient_valg = np.array([imu._rotm2eul(quaternion.as_rotation_matrix(q1)) for q1 in qorient_valg])
    #
    # imu.get_orientation(method='valenti', gain=0.5, gainrange=None)     # should be in between
    # qorient_valfix = copy(imu.qorient)
    # orient_valfix = np.array([imu._rotm2eul(quaternion.as_rotation_matrix(q1)) for q1 in qorient_valfix])
    # alphafix = copy(imu.alpha)
    #
    # imu.get_orientation(method='valenti', gain=1.0, gainrange=(0.0, 0.02))     # should be in between
    # qorient_valadapt = copy(imu.qorient)
    # orient_valadapt = np.array([imu._rotm2eul(quaternion.as_rotation_matrix(q1)) for q1 in qorient_valadapt])
    # alphaadapt = copy(imu.alpha)
    #
    # imu.get_orientation(method='integrate_gyro')     # should also be strict gyro integration
    # qorient_gyro = copy(imu.qorient)
    # orient_gyro = np.array([imu._rotm2eul(quaternion.as_rotation_matrix(q1)) for q1 in qorient_gyro])
    #
    # fig, ax = plt.subplots(5,1)
    # ax[0].plot(imu.t, np.rad2deg(orient_valg[:, 1]), 'b-',
    #            imu.t, np.rad2deg(imu.orient_imu_true[:, 1]), 'k--')
    # ax[0].set_ylabel('Gyro')
    #
    # ax[1].plot(imu.t, np.rad2deg(orient_vala[:, 1]), 'g-',
    #            imu.t, np.rad2deg(imu.orient_imu_true[:, 1]), 'k--')
    # ax[1].set_ylabel('Acc')
    #
    # ax[2].plot(imu.t, np.rad2deg(orient_valfix[:, 1]), 'r-',
    #            imu.t, np.rad2deg(imu.orient_imu_true[:, 1]), 'k--')
    # ax[2].set_ylabel('Fixed')
    #
    # ax[3].plot(imu.t, np.rad2deg(orient_valadapt[:, 1]), 'm-',
    #            imu.t, np.rad2deg(imu.orient_imu_true[:, 1]), 'k--')
    # ax[3].set_ylabel('Adapt')
    #
    # ax[4].plot(imu.t, np.linalg.norm(imu.acc, axis=1))
    # ax[4].set_ylabel('Acc mag')

    plt.show(block=True)

    # imu.filter(method='butter', order=5, gyro_cutoff=(0.1, 10), acc_cutoff=(0, 30))
    #
    # imu.get_orientation(method='madgwick')
    # orient_mad = np.rad2deg(copy(imu.orient_sensor))
    #
    # imu.filter(nsamp=10, method='running')
    #
    # imu.calibrate(duration=30.0)
    # imu.get_inertial_coords(duration=30.0)
    # # debug right handed coordinate system here
    #
    # fig, ax = plt.subplots(3,1)
    # for o1, ax1, lab in zip(np.rollaxis(orient_mad, 1), ax, ['roll', 'pitch', 'yaw']):
    #     ax1.plot(imu.t, o1, label='mad')
    #     ax1.set_ylabel(lab)
    # ax[0].set_title('Madgwick')
    #
    # imu.get_orientation(method='ekf', lCa=(0.0, -0.3, -0.7))
    # orient_ekf1 = np.rad2deg(copy(imu.orient_sensor))
    # accd1 = copy(imu.accdyn)
    #
    # fig, ax = plt.subplots(3,1)
    # for o1, ax1, lab in zip(np.rollaxis(orient_ekf1, 1), ax, ['roll', 'pitch', 'yaw']):
    #     ax1.plot(imu.t, np.unwrap(o1), label='ekf')
    #     ax1.set_ylabel(lab)
    # ax[0].set_title('EKF')
    #
    # fig, ax = plt.subplots(3,1)
    # for o1, o0, ax1 in zip(np.rollaxis(orient_ekf1, 1), np.rollaxis(imu.orient_imu_true, 1), ax):
    #     ax1.plot(imu.t, np.unwrap(o1), label='ekf')
    #     ax1.plot(imu.t, o0, 'k--', label='true')
    #
    # # fig, ax = plt.subplots(3,1)
    # # for o1, o0, ax1 in zip(np.rollaxis(orient_ekf1, 1), np.rollaxis(imu.orient_world_true, 1), ax):
    # #     ax1.plot(imu.t, np.unwrap(o1), label='ekf')
    # #     ax1.plot(imu.t, o0, 'k--', label='true')
    #
    # plt.show(block=True)

if __name__ == "__main__":
    main()




