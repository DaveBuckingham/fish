import numpy as np
#from scipy import interpolate, signal
from scipy import interpolate
from copy import copy

import itertools

import quaternion



class Am_process(object):

    def __init__(self):
        self.t = []
        self.gyro = []
        self.sampfreq = 200

        self.Qbias = None
        self.Qgyro = None
        self.Qacc = None
        self.Qdyn = None

        self.gN = np.array([0, 0, 1])


    def gram_schmidt(self, U):
        k = U.shape[1]
        V = copy(U)

        for i in range(k):
            V[:, i] /= np.linalg.norm(V[:, i])

            for j in range(i+1, k):
                proj = np.dot(V[:, i], V[:, j]) * V[:, i]
                V[:, j] -= proj

        return V

#   def gram_schmidt(self, X):
#       Q, R = np.linalg.qr(X)
#       return -Q


    def rotm2eul(self, rotm, singularity=0.001):
        phi = np.arctan2(rotm[1, 2], rotm[2, 2])
        theta = np.arcsin(rotm[0, 2])
        psi = np.arctan2(rotm[0, 1], rotm[0, 0])
        return phi, theta, psi








    def get_orientation_ekf(self, Ca, basis):
        """Extended Kalman filter for sensor fusion
        x is the state: [theta, bias, adyn]^T
        where theta are the Euler """

        dt = np.mean(np.diff(self.t))
        if Ca is None:
            Ca = np.power(10, np.array(lCa)) / dt
        else:
            Ca = np.array(Ca) / dt

        if self.Qbias is None or self.Qgyro is None or self.Qacc is None:
            raise ValueError('Noise covariance is not yet estimated')
        if self.gN is None:
            raise ValueError('Inertial reference frame is not yet set')

        self.Qdyn = np.diag(Ca).dot(self.Qacc)

        xkm1 = np.zeros((9,))
        dt = np.diff(self.t)
        dt = np.insert(dt, 0, dt[0:0])
        dt1 = dt[0]

        # make 9 x 9 matrix
        Pkm1 = self._stack_matrices([
            [(self.Qgyro + self.Qbias)*dt1**2,  -self.Qbias*dt1,    np.zeros((3, 3))],
            [-self.Qbias*dt1,                   self.Qbias,         np.zeros((3, 3))],
            [np.zeros((3, 3)),                  np.zeros((3, 3)),   self.Qdyn]])

        N = self.gyro.shape[0]

        gyro = np.deg2rad(self.gyro) - self.bias_gyro
        acc = 9.81*self.acc

        eulerEKF = []
        aD = []
        err = []
        PkEKF = []
        xkEKF = []
        Rk = self.Qacc

        for dt1, omegak, accel in zip(dt, gyro, acc):
            Fk, xkM, Bk = self._system_dynamics(xkm1, omegak, dt1, Ca)

            Qk = self._stack_matrices([
                [Bk.dot(self.Qgyro + self.Qbias).dot(Bk.T)*dt1**2,    -Bk.dot(self.Qbias)*dt1,  np.zeros((3,3))],
                [-self.Qbias.dot(Bk.T)*dt1,                           self.Qbias,               np.zeros((3,3))],
                [np.zeros((3,3)),                                     np.zeros((3,3)),          self.Qdyn]])

            PkM = Fk.dot(Pkm1).dot(Fk.T) + Qk
            hk, Jh = self._observation_dynamics(xkM, self.gN)

            Hk = Jh

            Sk = Hk.dot(PkM).dot(Hk.T) + Rk
            Kk = PkM.dot(Hk.T).dot(np.linalg.pinv(Sk))
            xk = xkM + Kk.dot(accel - hk)
            Pk = (np.eye(9) - Kk.dot(Hk)).dot(PkM)

            QT = self._eul2rotm(xk[:3])

            eulerEKF.append(xk[:3])
            aD.append(QT.T.dot(xk[6:]))
            err.append(accel - ((QT.dot(self.gN) + xk[6:])))
            PkEKF.append(Pk)
            xkEKF.append(xk)

            Pkm1 = Pk
            xkm1 = xk

        self.orient_sensor = np.pad(np.array(eulerEKF), ((1, 0), (0, 0)), mode='edge')
        self.accdyn_sensor = np.pad(np.array(aD), ((1, 0), (0, 0)), mode='edge')

        qorient = []
        for o1 in self.orient_sensor:
            qorient.append(quaternion.from_euler_angles(*o1))
        self.qorient = np.array(qorient)

        g0 = np.array([0, 0, 1.0])

        orient_world = []
        accdyn_world = []
        rotm_world = []
        qorient_world = []
        for chiprpy, adyn1 in zip(self.orient_sensor, self.accdyn_sensor):
            Rchip = self._eul2rotm(chiprpy)
            R = Rchip.dot(basis)

            worldrotm = basis.T.dot(R)
            orient_world.append(self.rotm2eul(worldrotm))
            qorient_world.append(self.rotm2quat(worldrotm))
            rotm_world.append(worldrotm)

            # rotate the dynamic acceleration into the world coordinates
            accdyn_world.append(R.T.dot(adyn1))

        self.orient_world = np.array(orient_world)
        self.orient_world_rotm = np.array(rotm_world)
        self.accdyn_sensor /= 9.81
        self.accdyn_world = np.array(accdyn_world) / 9.81
        self.accdyn = self.accdyn_world

        return self.orient_world




    def get_orientation_valenti(self, basis, initwindow=0.5, gain=0.5, gainrange=(0.1, 0.2), epsilon=0.9):
        def adapt_gain(gain, err, gainrange=(0.1, 0.2)):
            if err < gainrange[0]:
                return gain
            elif err < gainrange[1]:
                return gain * (1.0 - (err - gainrange[0]) / (gainrange[1] - gainrange[0]))
            else:
                return 0.0

        qchip2world = quaternion.from_rotation_matrix(basis)
        gyrorad = np.deg2rad(self.gyro)

        qorientGL = np.zeros_like(self.t, dtype=np.quaternion)

        qgyro = np.zeros_like(self.t, dtype=np.quaternion)
        gvec = np.zeros_like(self.gyro)

        dt = 1.0 / self.sampfreq

        isfirst = self.t <= self.t[0] + initwindow
        qorientGL[0] = qchip2world

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
        self.orient_sensor = np.array([self.rotm2eul(quaternion.as_rotation_matrix(qchip2world * q1))
                                       for q1 in qorient])
        self.accdyn_sensor = self.acc - gvec

        qorient_world = [qchip2world.conj() * q1.conj() for q1 in self.qorient]
        self.qorient_world = qorient_world

        self.orient_world_rotm = np.array([quaternion.as_rotation_matrix(q1) for q1 in qorient_world])
        self.orient_world = np.array([self.rotm2eul(R1) for R1 in self.orient_world_rotm])
        self.orient = self.orient_world

        # rotate accdyn into the world coordinate system
        qaccdyn_world = [qchip2world.conj() * np.quaternion(0, *a1) * qchip2world
                         for a1 in self.accdyn_sensor]
        self.accdyn_world = np.array([q.components[1:] for q in qaccdyn_world])
        self.accdyn = self.accdyn_world

        return self.orient_world



    def get_orientation_integrate_gyro(self, basis, initwindow=0.5):
        return get_orientation_madgwick(basis, initwindow=initwindow, beta=0)


    def get_orientation_madgwick(self, basis, data, initwindow=0.5, beta=2.86):
        qchip2world = quaternion.from_rotation_matrix(basis)
        timestamps = data.imu_data['timestamps']


        gyro = np.array([list(i) for i in zip(*data.imu_data['imus'][0]['gyro'])])
        accel = np.array([list(i) for i in zip(*data.imu_data['imus'][0]['accel'])])
        print(accel)
        exit()


        gyrorad = np.deg2rad(gyro)
        betarad = np.deg2rad(beta)

        qorient = np.zeros_like(timestamps, dtype=np.quaternion)
        qgyro = np.zeros_like(timestamps, dtype=np.quaternion)
        gvec = np.zeros_like(gyro)

        dt = 1.0 / self.sampfreq

        isfirst = self.t <= timestamps[0] + initwindow
        qorient[0] = qchip2world.conj()

        for i, gyro1 in enumerate(gyrorad[1:, :], start=1):
            qprev = qorient[i-1]

            acc1 = accel[i, :]
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

                J = np.array([[-2*qp[2],  2*qp[3], -2*qp[0], 2*qp[1]],
                               [2*qp[1],  2*qp[0],  2*qp[3], 2*qp[2]],
                               [0,       -4*qp[1], -4*qp[2], 0]])

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
        gvec = [(q.conj() * np.quaternion(0, 0, 0, 1) * q).components[1:] for q in qorient]

        self.orient_sensor = np.array([self.rotm2eul(quaternion.as_rotation_matrix(qchip2world * q1)) for q1 in qorient])
        self.accdyn_sensor = accel - gvec

        qorient_world = [qchip2world.conj() * q1.conj() for q1 in qorient]
        self.qorient_world = np.array(qorient_world)

        self.orient_world_rotm = np.array([quaternion.as_rotation_matrix(q1) for q1 in qorient_world])
        self.orient_world = np.array([self.rotm2eul(R1) for R1 in self.orient_world_rotm])
        self.orient = self.orient_world

        # rotate accdyn into the world coordinate system
        qaccdyn_world = [qchip2world.conj() * np.quaternion(0, *a1) * qchip2world
                         for a1 in self.accdyn_sensor]
        self.accdyn_world = np.array([q.components[1:] for q in qaccdyn_world])
        self.accdyn = self.accdyn_world

        return self.orient_world


    def calibrate(self, data):
        with h5py.File(filename, 'r') as h5calib:
            gyro = np.array(h5calib['/data/Gyro'])
            # convert file data from deg/sec to rad/sec
            gyro = np.deg2rad(gyro)

            accel = 9.81 * np.array(h5calib['/data/Accel'])

            self._calibrate(accel, gyro)

    def _calibrate(self, accel, gyro):
        """Get initial covariances.
        gyro in rad/sec
        accel in m/s^2"""
        self.bias_gyro = np.mean(gyro, axis=0)

        # get noise covariances
        self.Qgyro = np.cov(gyro, rowvar=False)
        self.Qacc = np.cov(accel, rowvar=False)

        # bias noise covariance (assuming low drift)
        self.Qbias = 1e-10 * self.Qacc

        # gyro noise
        self.gyro_noise = np.std(gyro, axis=0)


