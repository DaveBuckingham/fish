import os, sys
import h5py
import numpy as np

import matplotlib.pyplot as plt

def main():
    plt.ion()

    datadir = '/Users/etytel01/Documents/Acceleration/rawdata/data_2017_07_28'
    zplus_axis_file = 'static_straight_up.hdf5'
    xplus_axis_file = 'static_laying_flat_reversed.hdf5'
    yneg_axis_file = 'static_sticking_out.hdf5'
    axsign = [1, -1, 1]
    calib_file = 'static_straight_up.hdf5'

    outfile = 'calib.hdf5'

    datasets = ['Accel', 'Accel2', 'Encoder', 'Gyro', 'Gyro2', 'Mag', 'Mag2']
    outdatasets = ['accel1', 'accel2', 'Encoder', 'gyro1', 'gyro2', 'mag1', 'mag2']

    data = [np.array([]) for _ in datasets]
    t = np.array([], dtype='int64')
    tprev = np.int64(0)

    for fn, axsign1 in zip([xplus_axis_file, yneg_axis_file, zplus_axis_file], axsign):
        fn = os.path.join(datadir, fn)

        with h5py.File(os.path.join(datadir, fn), 'r') as f:
            g = f['/data']
            for i, dsname in enumerate(datasets):
                ds1 = np.array(g[dsname])

                if dsname in ['accel1', 'accel2']:
                    ds1 *= axsign1

                if data[i].size == 0:
                    data[i] = ds1
                else:
                    data[i] = np.concatenate((data[i], ds1))

            t1 = np.array(g['t'])
            t1 += tprev
            tprev = np.max(t1)

            t = np.concatenate((t, t1))

    with h5py.File(os.path.join(datadir, outfile), 'w') as f:
        g = f.create_group('/data')

        for data1, dsname1 in zip(data, outdatasets):
            g.create_dataset(dsname1, data=data1)

        g.create_dataset('time', data=t)

    fig, ax = plt.subplots(3, 1)
    ax[0].plot(data[0])
    ax[1].plot(data[2])
    ax[2].plot(data[3])
    plt.show(block=True)


if __name__ == "__main__":
    main()
