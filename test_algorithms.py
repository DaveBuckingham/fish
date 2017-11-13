import numpy as np
from scipy import signal

from imucapture.ic_process import eul2rotm, rotm2eul

class test_data():
    def __init__(self, duration, basefreq, accelamp, orientamp, chip2world, sampfreq=200.0, pad=1.0, timescale=1000.0):
        self.sampfreq = sampfreq
        self.duration = duration
        self.timescale = timescale

        self.basefreq = basefreq
        self.num_imus = 1

        t = np.arange(0, duration + 2 * pad, 1.0 / sampfreq)
        t = t - pad
        self.t = t

        T = 0.25 / (2 * basefreq)

        world2chip = chip2world.T

        try:
            accel = []
            for a1 in accelamp:
                j0 = a1 / T
                amp1 = 1 / 3 * j0 * T ** 3

                _, _, accel1 = self.make_wave(t, 2*basefreq, amp1, duration)
                accel.append(accel1)
            accel = np.array(accel).T
        except TypeError:
            j0 = accelamp / T
            amp = 1 / 3 * j0 * T ** 3

            _, _, accelx = self.make_wave(t, 2 * basefreq, amp, duration)
            accel = np.zeros((len(accelx), 3))
            accel[:, 0] = accelx

        # set up the roll, pitch, and yaw
        orient = []
        orientrate = []
        for oa1, f1 in zip(orientamp, [basefreq, 2*basefreq, basefreq]):
            o = oa1 * np.sin(2*np.pi * f1 * t)
            o[t < 0] = 0.0
            o[t > duration] = 0.0
            orient.append(o)

            orate = 2*np.pi * f1 * oa1 * np.cos(2*np.pi * f1 * t)
            orate[t < 0] = 0.0
            orate[t > duration] = 0.0
            orientrate.append(orate)

        # roll, rollrate, _ = self.make_wave(t, basefreq, orientamp[0], duration)
        # pitch, pitchrate, _ = self.make_wave(t, 2 * basefreq, orientamp[1], duration)
        # yaw, yawrate, _ = self.make_wave(t, basefreq, orientamp[2], duration)

        self.accdyn = accel
        self.accdyn_chip = np.array([world2chip.dot(a1) for a1 in accel])
        # self.orient = np.vstack((roll, pitch, yaw)).T
        # self.orientrate = np.vstack((rollrate, pitchrate, yawrate)).T
        self.orient = np.array(orient).T
        self.orientrate = np.array(orientrate).T

        # get the angular velocities
        self.omega_world = np.array([self.get_angular_velocity(u, udot) \
                                     for u, udot in zip(self.orient, self.orientrate)])
        self.omega_chip0 = np.array([world2chip.dot(omega1) for omega1 in self.omega_world])

        self.rotm = np.array([eul2rotm(u) for u in self.orient])

        g0 = np.array([0, 0, 9.81])
        g = np.array([R1.dot(g0) for R1 in self.rotm])

        self.g_chip = np.array([world2chip.dot(g1) for g1 in g])

        self.acc_chip0 = self.g_chip + self.accdyn_chip

    def add_noise(self, accrms, acclofreq, gyrorms, gyrolofreq, drift, drifthifreq):
        sampfreq = self.sampfreq

        # accelerometer noise

        if accrms > 0:
            n = np.random.standard_normal(self.acc_chip0.shape)

            # no covariance among channels
            sos = signal.butter(9, acclofreq / (sampfreq / 2), btype='highpass', output='sos')
            acc_noise = signal.sosfilt(sos, n, axis=0)
            acc_noise /= np.sqrt(np.mean(np.square(acc_noise), axis=0))
            acc_noise *= accrms
        else:
            acc_noise = np.zeros_like(self.acc_chip0)

        self.acc = self.acc_chip0 + acc_noise

        # gyro noise

        if gyrorms > 0:
            # first general noise
            n = np.random.standard_normal(self.omega_chip0.shape)

            sos = signal.butter(9, gyrolofreq / (sampfreq / 2), btype='highpass', output='sos')
            gyro_noise = signal.sosfilt(sos, n, axis=0)
            gyro_noise /= np.sqrt(np.mean(np.square(gyro_noise), axis=0))
            gyro_noise *= gyrorms
        else:
            gyro_noise = np.zeros_like(self.omega_chip0)

        # then drift
        if drift > 0.0:
            n = np.random.standard_normal(self.omega_chip0.shape)

            sos = signal.butter(9, drifthifreq / (sampfreq / 2), btype='lowpass', output='sos')
            gyro_drift = signal.sosfilt(sos, n, axis=0)
            gyro_drift /= np.sqrt(np.mean(np.square(gyro_drift), axis=0))
            gyro_drift *= drift
        else:
            gyro_drift = np.zeros_like(self.omega_chip0)

        self.gyro = self.omega_chip0 + gyro_noise + gyro_drift

        # duck typing for Ic_data
        self.imu_data = {}
        self.imu_data['timestamps'] = self.t * self.timescale

        self.imu_data['imus'] = [{'accel': [self.acc[:, 0], self.acc[:, 1], self.acc[:, 2]],
                                  'gyro': [self.gyro[:, 0], self.gyro[:, 1], self.gyro[:, 2]]}]

    def num_samples(self):
        return len(self.t)

    def as_list_of_triples(self, imu_index, mode):
        return list(zip(*self.imu_data['imus'][imu_index][mode]))


    def make_wave(self, t, freq, amp, duration):
        """Makes a snap-free approximation of a sine wave and its derivatives"""
        T = 0.25 / freq
        j0 = 3 * amp / T ** 3
        a0 = j0 * T
        v0 = 0.5 * j0 * T ** 2
        x0 = amp

        ph = np.mod(t * freq, 1.0)
        ph[t < 0] = -1
        ph[t > duration] = -1

        pos = np.zeros_like(t)
        vel = np.zeros_like(t)
        accel = np.zeros_like(t)

        isint = np.logical_and(ph >= 0, ph < 0.25)
        tint = (ph[isint] - 0.0) / freq
        np.place(pos, isint, v0 * tint - 1 / 6 * j0 * np.power(tint, 3))
        np.place(vel, isint, v0 - 0.5 * j0 * np.power(tint, 2))
        np.place(accel, isint, -j0 * tint)

        isint = np.logical_and(ph >= 0.25, ph < 0.5)
        tint = (ph[isint] - 0.25) / freq
        np.place(pos, isint, x0 - 0.5 * a0 * np.power(tint, 2) + 1 / 6 * j0 * np.power(tint, 3))
        np.place(vel, isint, -a0 * tint + 0.5 * j0 * np.power(tint, 2))
        np.place(accel, isint, -a0 + j0 * tint)

        isint = np.logical_and(ph >= 0.5, ph < 0.75)
        tint = (ph[isint] - 0.5) / freq
        np.place(pos, isint, -v0 * tint + 1 / 6 * j0 * np.power(tint, 3))
        np.place(vel, isint, -v0 + 0.5 * j0 * np.power(tint, 2))
        np.place(accel, isint, j0 * tint)

        isint = np.logical_and(ph >= 0.75, ph < 1.0)
        tint = (ph[isint] - 0.75) / freq
        np.place(pos, isint, -x0 + 0.5 * a0 * np.power(tint, 2) - 1 / 6 * j0 * np.power(tint, 3))
        np.place(vel, isint, a0 * tint - 0.5 * j0 * np.power(tint, 2))
        np.place(accel, isint, a0 - j0 * tint)

        return pos, vel, accel

    def get_angular_velocity(self, u, udot):
        """Constructs the angular velocity vector from Euler angles and Euler angle rates"""
        (phi, theta, psi) = u

        E123 = np.array([[np.cos(theta)*np.cos(phi),     -np.sin(phi),      0],
                         [np.cos(theta)*np.sin(phi),      np.cos(phi),      0],
                         [-np.sin(theta),                 0,                1]])
        omega = E123.dot(udot)

        return omega

class calibration_data(test_data):
    def __init__(self, duration, chip2world, sampfreq=200.0, timescale=1000.0):
        self.sampfreq = sampfreq
        self.duration = duration
        self.num_imus = 1
        self.timescale = timescale

        t = np.arange(0, duration, 1.0 / sampfreq)
        self.t = t

        T = duration/3

        world2chip = chip2world.T

        g0 = np.array([0, 0, 9.81])

        R1 = eul2rotm((0, -np.pi/2, 0))
        g1 = world2chip.dot(R1.dot(g0))

        R2 = eul2rotm((np.pi/2, 0, 0))
        g2 = world2chip.dot(R2.dot(g0))

        R3 = np.eye(3)
        g3 = world2chip.dot(R3.dot(g0))

        acc = np.zeros((len(t), 3))
        acc[t < T, :] = g1[np.newaxis, :]
        acc[np.logical_and(t >= T, t < 2*T), :] = g2[np.newaxis, :]
        acc[np.logical_and(t >= 2*T, t < duration), :] = g3[np.newaxis, :]

        self.acc_chip0 = acc
        self.omega_chip0 = np.zeros_like(acc)

