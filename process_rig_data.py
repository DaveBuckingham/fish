import numpy as np
from scipy import interpolate

import os, sys

import logging

from PyQt5 import QtWidgets, QtCore
import matplotlib.pyplot as plt

from copy import copy

import h5py

import seaborn as sns
sns.set_context('notebook', font_scale=1.5)

from imucapture.transform import Transform, eul2rotm
from imucapture.data import Data
from imucapture.calib import Calib

timescale = 1000.0

def main():
    global timescale

    logging.basicConfig(level=logging.DEBUG)

    calibdata = Data.from_file_smart('/Users/etytel01/Documents/Acceleration/rawdata/data_2017_07_28/calib.hdf5')
    calibdata1 = calibdata.get_one_imu(0)

    calib = Calib()
    calib.parse_data(calibdata1)

    world_basis = calib.imu_bases[0]

    datafilename = '/Users/etytel01/Documents/Acceleration/rawdata/data_2017_07_28/rotate_only.hdf5'
    data = Data.from_file_smart(datafilename, resample=True)

    t = data.timestamps/timescale

    acc1 = data.get_acceleration(0)
    acc1 = acc1[:, t < 0.5]
    mag = np.linalg.norm(acc1, axis=0)

    acc1 = acc1 / mag

    initial_gravity = np.mean(acc1, axis=1) * 9.81
    calib.initial_gravity = initial_gravity

    plt.ion()

    fig, ax = plt.subplots()

    acc1 = data.get_acceleration(0)
    mag = np.linalg.norm(acc1, axis=0)

    ax.plot(t, data.get_acceleration(0).T)
    ax.plot(t, mag)

    transform = Transform()

    accdyn_mad, orient_mad = transform.get_orientation_madgwick(data, calib, imu=0, filter_num_samples=10,
                                                                beta=np.deg2rad(2.86))
    accdyn_dsf, orient_dsf = transform.get_orientation_dsf(data, calib, imu=0, filter_num_samples=10,
                                                   accdynmag=200.0)


    fig, ax = plt.subplots(3, 1)

    for ax1, acc1 in zip(ax, accdyn_mad):
        ax1.plot(t, acc1, label='mad')
    for ax1, acc1 in zip(ax, accdyn_dsf):
        ax1.plot(t, acc1, label='dsf')

    g = []
    for chiprpy in np.rollaxis(orient_mad, 1):
        Rchip = eul2rotm(chiprpy)
        g1 = Rchip.dot(calib.initial_gravity)
        g.append(g1)
    g = np.array(g)

    with h5py.File(datafilename) as h5file:
        orient_y = np.deg2rad(np.array(h5file['/data/Encoder']))

    # filter with a running average
    orient_y = np.convolve(orient_y, np.ones((80,))/80, mode='same')

    orient_true = np.zeros_like(orient_mad)
    orient_true[1, :] = orient_y
    orient_true[1, :] = orient_true[1, :] - orient_true[1, 0]

    omega = data.get_gyroscope(0)
    dt = np.mean(np.diff(t))

    omega_dot = np.gradient(omega, dt, axis=1)

    omega_world_true = -np.gradient(orient_true, dt, axis=1)
    omega_dot_world_true = np.gradient(omega_world_true, dt, axis=1)

    omega_chip_true = world_basis.T.dot(omega_world_true)

    armlen = 0.5    # meters
    xpb_world = np.array([0, 0, armlen])
    xpb_chip = world_basis.T.dot(xpb_world)


    # CONTINUE here - debug math
    acc_angular = world_basis.dot(np.cross(omega_dot, xpb_chip).T).T
    acc_centrip = world_basis.dot(np.cross(omega, np.cross(omega, xpb_chip)).T).T

    acc_base = np.array(data.as_list_of_triples(1, 'accel'))

    acc_angular_true = np.cross(omega_dot_world_true, xpb_world)
    acc_centrip_true = np.cross(omega_world_true, np.cross(omega_world_true, xpb_world))
    acc_dyn_true = acc_angular_true + acc_centrip_true
    acc_dyn_true[:, 0] += acc_base[:, 0]


    fig, ax = plt.subplots(3)
    for ab1, ax1 in zip(np.rollaxis(acc_base, 1), ax):
        ax1.plot(t, ab1)

    fig, ax = plt.subplots(3,2)
    for acc1, acc_true1, orient1, orient_true1, ax1 in zip(np.rollaxis(np.array(accdyn_mad), 1), np.rollaxis(acc_dyn_true, 1),
                                                           np.rollaxis(np.array(orient_mad), 1),
                                  np.rollaxis(orient_true, 1), ax):
        ax1[0].plot(t, acc_true1, 'k--', label='true')
        ax1[0].plot(t, acc1, label='imu1')
        ax1[1].plot(t, np.rad2deg(orient_true1), 'k--', label='true')
        ax1[1].plot(t, np.rad2deg(orient1), label='imu1')

    ax[0,1].legend(loc='upper right')
    ax[0,0].set_title('Dynamic acceleration')
    ax[0,1].set_title('Orientation')
    ax[0,0].set_ylabel('Xworld')
    ax[1,0].set_ylabel('Yworld')
    ax[2,0].set_ylabel('Zworld')

    ax[2,0].set_xlabel('Time (sec)')
    ax[2,1].set_xlabel('Time (sec)')

    logging.info('Check plots!')
    plt.show(block=True)

if __name__ == "__main__":
    main()
