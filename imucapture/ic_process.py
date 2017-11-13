import numpy
from scipy import interpolate, signal
import quaternion
import h5py
from copy import copy
import re


class Ic_process():
    def __init__(self):
        pass


    def filter(self, data, nsamp):
        filtered = numpy.zeros_like(data)
        for i in range(3):
            filtered[:, i] = numpy.convolve(data[:, i], numpy.ones((nsamp,))/nsamp, mode='same')
        return filtered




    def get_chip_to_world_rotation_matrix(self, acc, time):
        # TODO: I'm pretty sure this function should not be used.  The matrix should be the basis vectors from ic_get_basis.get_basis_vector
        z_axis_index = 2

        averagedur = 4*time[-1]

        ist = numpy.logical_and(time >= -(averagedur/2), time <= (averagedur/2))
        ax = numpy.mean(acc[ist, :], axis=0)

        gax = numpy.eye(3)
        gax[:, z_axis_index] = ax

        axord = [z_axis_index]
        axord = numpy.concatenate((axord, numpy.setdiff1d(range(3), axord)))
        axrevord = numpy.argsort(numpy.arange(3)[axord])

        basis = gramschmidt(gax[:, axord])
        basis = basis[:, axrevord]

        # CHECK FOR RIGHT HANDED-NESS
        assert(numpy.dot(numpy.cross(basis[:, 0], basis[:, 1]), basis[:, 2]) > 0.9)

        return basis



    
    def get_orientation_integrate(self, chip2world_rot, data, filter_num_samples, initwindow=0.5):
        return self.get_orientation_madgwick(chip2world_rot, data, filter_num_samples, initwindow=initwindow, beta=0.0)




    def get_orientation_dsf(self, Ca_arg, initial_gravity, chip2world_rot, still_accel, still_gyro, data, filter_num_samples,
                            accdynmag=200.0):
        """Dynamic snap free Kalman filter for sensor fusion
        x is the state: [theta, bias, adyn, jerk]^T
        where theta are the Euler angles"""

        # TODO: Ca_arg parameter is no longer needed.  Remove

        # GET ACC IN MPS2
        acc  = numpy.array(data.as_list_of_triples(0, 'accel'))

        # GET GYRO IN RADIANS PER SEC
        gyro = numpy.array(data.as_list_of_triples(0, 'gyro'))

        # FILTER DATA
        acc  = self.filter(acc, filter_num_samples)
        gyro = self.filter(gyro, filter_num_samples)

        # GET TIME IN MS
        time = numpy.array(data.imu_data['timestamps'])

        # CONVERT TIME TO SECONDS
        time = time / 1000.0




        bias_gyro = numpy.mean(still_gyro, axis=0)

        # GET NOISE COVARIANCES
        Qgyro = numpy.cov(still_gyro, rowvar=False)
        Qacc = numpy.cov(still_accel, rowvar=False)

        # BIAS NOISE COVARIANCE (ASSUMING LOW DRIFT)
        Qbias = 1e-10 * Qacc

        Qdyn = accdynmag * stack_matrices([[Qacc, numpy.zeros((3, 3))],
                                                 [numpy.zeros((3, 3)), Qacc]])

        xkm1 = numpy.zeros((12,))
        dt = numpy.diff(time)
        dt = numpy.insert(dt, 0, dt[0:0])
        dt1 = dt[0]

        # MAKE 12 X 12 MATRIX
        Pkm1 = stack_matrices([
            [(Qgyro + Qbias)*dt1**2,  -Qbias*dt1,               numpy.zeros((3, 6))],
            [-Qbias*dt1,               Qbias,                   numpy.zeros((3, 6))],
            [numpy.zeros((6, 3)),      numpy.zeros((6, 3)),     Qdyn]])

        gyro_unbiased = gyro - bias_gyro

        eulerEKF = []
        aD = []
        err = []
        PkEKF = []
        xkEKF = []
        Rk = Qacc

        for dt1, omegak, accel in zip(dt, gyro_unbiased, acc):
            Fk, xkM, Bk = self._system_dynamics(xkm1, omegak, dt1)

            Qk = stack_matrices([
                [Bk.dot(Qgyro + Qbias).dot(Bk.T)*dt1**2,  -Bk.dot(Qbias)*dt1,  numpy.zeros((3, 6))],
                [-Qbias.dot(Bk.T)*dt1,                     Qbias,              numpy.zeros((3, 6))],
                [numpy.zeros((6,6)),                                           Qdyn]])

            PkM = Fk.dot(Pkm1).dot(Fk.T) + Qk
            hk, Jh = self._observation_dynamics(xkM, initial_gravity)

            Hk = Jh

            Sk = Hk.dot(PkM).dot(Hk.T) + Rk
            Kk = PkM.dot(Hk.T).dot(numpy.linalg.pinv(Sk))
            xk = xkM + Kk.dot(accel - hk)
            Pk = (numpy.eye(12) - Kk.dot(Hk)).dot(PkM)

            QT = eul2rotm(xk[:3])

            eulerEKF.append(xk[:3])
            aD.append(QT.T.dot(xk[6:9]))
            err.append(accel - ((QT.dot(initial_gravity) + xk[6:9])))

            PkEKF.append(Pk)
            xkEKF.append(xk)

            Pkm1 = Pk
            xkm1 = xk

        orient_sensor = numpy.pad(numpy.array(eulerEKF), ((1, 0), (0, 0)), mode='edge')
        accdyn_sensor = numpy.pad(numpy.array(aD), ((1, 0), (0, 0)), mode='edge')

        orient_world = []
        accdyn_world = []
        rotm_world = []
        for chiprpy, adyn1 in zip(orient_sensor, accdyn_sensor):
            Rchip = eul2rotm(chiprpy)
            R = Rchip.dot(chip2world_rot)

            rotm_world1 = chip2world_rot.T.dot(R)
            orient_world.append(rotm2eul(rotm_world1))
            rotm_world.append(rotm_world1)

            # rotate the dynamic acceleration into the world coordinates
            accdyn_world.append(R.T.dot(adyn1))

        return accdyn_world, orient_world, rotm_world

    def get_orientation_madgwick(self, chip2world_rot, data, filter_num_samples, initwindow=0.5, beta=2.86):

        # GET ACC IN MPS2
        acc  = numpy.array(data.as_list_of_triples(0, 'accel'))

        # GET GYRO IN RADIANS PER SEC
        gyro = numpy.array(data.as_list_of_triples(0, 'gyro'))

        # FILTER DATA
        acc  = self.filter(acc, filter_num_samples)
        gyro = self.filter(gyro, filter_num_samples)

        # CONVERT ACCEL DATA TO GS
        acc = acc / 9.81

        # GET TIME IN MS
        time = numpy.array(data.imu_data['timestamps'])

        # CONVERT TIME TO SECONDS
        time = time / 1000.0

        qchip2world = quaternion.from_rotation_matrix(chip2world_rot)

        qorient = numpy.zeros_like(time, dtype=numpy.quaternion)

        sampfreq = 1.0/numpy.mean(numpy.diff(time))

        dt = 1.0 / sampfreq

        qorient[0] = qchip2world.conj()

        for i, gyro1 in enumerate(gyro[1:, :], start=1):
            qprev = qorient[i-1]

            acc1 = acc[i, :]
            acc1 = acc1 / numpy.linalg.norm(acc1)

            # QUATERNION ANGULAR CHANGE FROM THE GRYO
            qdotgyro = 0.5 * (qprev * numpy.quaternion(0, *gyro1))

            if beta > 0:
                # GRADIENT DESCENT ALGORITHM CORRECTIVE STEP
                qp = qprev.components
                F = numpy.array([2*(qp[1]*qp[3] - qp[0]*qp[2]) - acc1[0],
                              2*(qp[0]*qp[1] + qp[2]*qp[3]) - acc1[1],
                              2*(0.5 - qp[1]**2 - qp[2]**2) - acc1[2]])
                J = numpy.array([[-2*qp[2], 2*qp[3], -2*qp[0], 2*qp[1]],
                               [2*qp[1], 2*qp[0], 2*qp[3], 2*qp[2]],
                               [0, -4*qp[1], -4*qp[2], 0]])

                step = numpy.dot(J.T, F)
                step = step / numpy.linalg.norm(step)
                step = numpy.quaternion(*step)

                qdot = qdotgyro - numpy.deg2rad(beta) * step
            else:
                qdot = qdotgyro

            qorient[i] = qprev + qdot * dt
            qorient[i] /= numpy.abs(qorient[i])

        # GET THE GRAVITY VECTOR
        # GRAVITY IS +Z
        gvec = [(q.conj() * numpy.quaternion(0, 0, 0, 1) * q).components[1:] for q in qorient]
        accdyn_sensor = acc - gvec

        # ATTEMPT TO CONVERT FROM CHIP COORDINATES TO WORLD
        # qorient IS THE QUATERNION THAT SPECIFIES THE CURRENT ORIENTATION OF THE CHIP, RELATIVE TO ITS INITIAL ORIENTATION
        # qchip2world IS THE QUATERNION THAT ROTATES FROM THE INITIAL CHIP ORIENTATION TO THE WORLD FRAME
        qorient_world = [qchip2world.conj() * q1.conj() for q1 in qorient]
        orient_world_rotm = [quaternion.as_rotation_matrix(q1) for q1 in qorient_world]
        orient_world = [rotm2eul(R1) for R1 in orient_world_rotm]


        # ROTATE ACCDYN INTO THE WORLD COORDINATE SYSTEM
        qaccdyn_world = [(qchip2world.conj() * numpy.quaternion(0, *a1) * qchip2world) for a1 in accdyn_sensor]
        accdyn_world = [q.components[1:] for q in qaccdyn_world]

        # CONVERT ACCEL DATA BACK TO MPS2
        accdyn_world = [i * 9.81 for i in accdyn_world]

        return (accdyn_world, orient_world, orient_world_rotm)








    def _system_dynamics(self, xk, omegak, dt):
        phi, theta, psi = xk[:3]
        biask = xk[3:6]

        sPh = numpy.sin(phi)
        cPh = numpy.cos(phi)
        tTh = numpy.tan(theta)
        scTh = 1 / numpy.cos(theta)

        Bk     = numpy.array([[1,  sPh*tTh,    cPh*tTh  ],
                              [0,  cPh,       -sPh      ],
                              [0,  sPh*scTh,   cPh*scTh]])

        # partial diffs
        Bk_phi = numpy.array([[0,  cPh*tTh,    -sPh*tTh ],
                              [0, -sPh,        -cPh     ],
                              [0,  cPh*scTh,   -sPh*scTh]])

        Bk_theta = numpy.array([[0,    sPh*scTh**2,    cPh*scTh**2 ],
                                [0,    0,              0           ],
                                [0,    sPh*scTh*tTh,   cPh*scTh*tTh]])

        Bk_psi = numpy.zeros((3, 3))

        unbiased_omegak = omegak - biask
        unbiased_omegak = unbiased_omegak[:, numpy.newaxis]

        Bkomega = numpy.hstack((numpy.dot(Bk_phi, unbiased_omegak),
                                numpy.dot(Bk_theta, unbiased_omegak),
                                numpy.dot(Bk_psi, unbiased_omegak)))

        jerk = xk[9:]

        Fk = stack_matrices([
            [numpy.eye(3) + Bkomega*dt, -Bk*dt, numpy.zeros((3, 6))],
            [numpy.zeros((3, 3)), numpy.eye(3), numpy.zeros((3, 6))],
            [numpy.zeros((3, 6)), numpy.eye(3), numpy.eye(3)*dt],
            [numpy.zeros((3, 6)), numpy.zeros((3, 3)), numpy.eye(3)]])

        xkp1 = numpy.hstack((xk[:3] + numpy.dot(Bk, unbiased_omegak).squeeze()*dt, xk[3:6],
                             xk[6:9] + jerk*dt, xk[9:]))

        return Fk, xkp1, Bk


    def _observation_dynamics(self, xk, gN):
        """gN = gravity in inertial coordinate system (3x1)"""
        phi, theta, psi = xk[:3]

        # rotation matrices
        Rz_yaw =    numpy.array([[numpy.cos(psi),     numpy.sin(psi),    0],
                             [-numpy.sin(psi),     numpy.cos(psi),    0],
                             [0,                0,              1]])
        Ry_pitch = numpy.array([[numpy.cos(theta),    0,              -numpy.sin(theta)],
                             [0,                1,              0],
                             [numpy.sin(theta),    0,              numpy.cos(theta)]])
        Rx_roll =  numpy.array([[1,                0,              0],
                             [0,                numpy.cos(phi),    numpy.sin(phi)],
                             [0,                -numpy.sin(phi),   numpy.cos(phi)]])

        # rates/derivatives
        Rz_yaw_rate =   numpy.array([[-numpy.sin(psi),    numpy.cos(psi),    0],
                                  [-numpy.cos(psi),    -numpy.sin(psi),   0],
                                  [0,               0,              0]])
        Ry_pitch_rate = numpy.array([[-numpy.sin(theta),  0,              -numpy.cos(theta)],
                                  [0,               0,              0],
                                  [numpy.cos(theta),   0,              -numpy.sin(theta)]])
        Rx_roll_rate =  numpy.array([[0,               0,              0],
                                  [0,               -numpy.sin(phi),   numpy.cos(phi)],
                                  [0,               -numpy.cos(phi),   -numpy.sin(phi)]])

        QT       = Rx_roll.dot(Ry_pitch).dot(Rz_yaw)
        QT_roll  = Rx_roll_rate.dot(Ry_pitch).dot(Rz_yaw)
        QT_pitch = Rx_roll.dot(Ry_pitch_rate).dot(Rz_yaw)
        QT_yaw   = Rx_roll.dot(Ry_pitch).dot(Rz_yaw_rate)

        Jh = numpy.vstack((QT_roll.dot(gN), QT_pitch.dot(gN), QT_yaw.dot(gN), numpy.zeros((3, 3)),
                           numpy.eye(3), numpy.zeros((3, 3)))).T
        hk = QT.dot(gN) + xk[6:9]

        return hk, Jh


def stack_matrices(M):
    m = []
    for row in M:
        m.append(numpy.hstack(tuple(row)))
    return numpy.vstack(tuple(m))



def gramschmidt(U):
    k = U.shape[1]
    V = copy(U)

    for i in range(k):
        V[:, i] /= numpy.linalg.norm(V[:, i])

        for j in range(i+1, k):
            proj = numpy.dot(V[:, i], V[:, j]) * V[:, i]
            V[:, j] -= proj

    return V


def eul2rotm(x):
    (phi, theta, psi) = x

    Rz_yaw =    numpy.array([[numpy.cos(psi),     numpy.sin(psi),    0],
                         [-numpy.sin(psi),     numpy.cos(psi),    0],
                         [0,                0,              1]])
    Ry_pitch = numpy.array([[numpy.cos(theta),    0,              -numpy.sin(theta)],
                         [0,                1,              0],
                         [numpy.sin(theta),    0,              numpy.cos(theta)]])
    Rx_roll =  numpy.array([[1,                0,              0],
                         [0,                numpy.cos(phi),    numpy.sin(phi)],
                         [0,                -numpy.sin(phi),   numpy.cos(phi)]])
    return Rx_roll.dot(Ry_pitch.dot(Rz_yaw))


def rotm2eul(rotm, singularity=0.001):
    phi   =  numpy.arctan2(rotm[1, 2], rotm[2, 2])
    theta =  -numpy.arcsin(rotm[0, 2])
    psi   =  numpy.arctan2(rotm[0, 1], rotm[0, 0])
    return (phi, theta, psi)



