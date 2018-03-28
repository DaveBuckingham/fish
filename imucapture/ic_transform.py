import numpy

#import quaternion
from pyquaternion import Quaternion

from copy import copy

from imucapture.ic_global import *
from imucapture.ic_data import Ic_data


class Ic_transform():
    def __init__(self):
        pass




    def filter(self, data, nsamp):
        filtered = numpy.zeros_like(data)
        for i in range(3):
            filtered[:, i] = numpy.convolve(data[:, i], numpy.ones((nsamp,))/nsamp, mode='same')
        return filtered




    
    def get_orientation_integrate(self, data, calib, imu, filter_num_samples, initwindow=0.5):
        return self.get_orientation_madgwick(data, calib, imu, filter_num_samples, initwindow=initwindow, beta=0.0)



    def get_orientation_dsf(self, data, calib, imu, filter_num_samples, accdynmag=200.0):
        # Dynamic snap free Kalman filter for sensor fusion
        # x is the state: [theta, bias, adyn, jerk]^T
        # where theta are the Euler angles

        # GET ACC IN MPS2
        acc = data.imu_data[imu, Ic_data.ACCEL_INDEX, :, :].transpose()

        # GET GYRO IN RADIANS PER SEC
        gyro = data.imu_data[imu, Ic_data.GYRO_INDEX, :, :].transpose()

        # FILTER DATA
        acc  = self.filter(acc, filter_num_samples)
        gyro = self.filter(gyro, filter_num_samples)

        # GET TIME IN MS
        #time = numpy.array(data.imu_data['timestamps'])

        # CONVERT TIME TO SECONDS
        #time = time / 1000.0

        # COMPUTE TIME ARRAY IN SECONDS, ASSUME GLOBAL VALUE IS CORRECT
        time = numpy.array(numpy.arange(0, data.num_samples*Ic_global.SECONDS_PER_SAMPLE, Ic_global.SECONDS_PER_SAMPLE))


        bias_gyro = numpy.mean(calib.still_gyro, axis=0)

        # GET NOISE COVARIANCES
        Qgyro = numpy.cov(calib.still_gyro, rowvar=False)
        Qacc = numpy.cov(calib.still_accel, rowvar=False)

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
            hk, Jh = self._observation_dynamics(xkM, calib.initial_gravity)

            Hk = Jh

            Sk = Hk.dot(PkM).dot(Hk.T) + Rk
            Kk = PkM.dot(Hk.T).dot(numpy.linalg.pinv(Sk))
            xk = xkM + Kk.dot(accel - hk)
            Pk = (numpy.eye(12) - Kk.dot(Hk)).dot(PkM)

            QT = eul2rotm(xk[:3])

            eulerEKF.append(xk[:3])
            aD.append(QT.T.dot(xk[6:9]))
            err.append(accel - ((QT.dot(calib.initial_gravity) + xk[6:9])))

            PkEKF.append(Pk)
            xkEKF.append(xk)

            Pkm1 = Pk
            xkm1 = xk

        orient_sensor = numpy.pad(numpy.array(eulerEKF), ((1, 0), (0, 0)), mode='edge')
        accdyn_sensor = numpy.pad(numpy.array(aD), ((1, 0), (0, 0)), mode='edge')

#        orient_world = []
#        accdyn_world = []
#        #rotm_world = []
#
#        for chiprpy, adyn1 in zip(orient_sensor, accdyn_sensor):
#            Rchip = eul2rotm(chiprpy)
#            R = Rchip.dot(calib.imu_bases[imu])
#
#            rotm_world1 = calib.imu_bases[imu].T.dot(R)
#            orient_world.append(rotm2eul(rotm_world1))
#            #rotm_world.append(rotm_world1)
#
#            # rotate the dynamic acceleration into the world coordinates
#            accdyn_world.append(R.T.dot(adyn1))
#
#        #return (accdyn_world, orient_world, rotm_world)
#        return (accdyn_world, orient_world)

        accdyn_world = numpy.empty([3,0])
        orient_world = numpy.empty([3,0])
        for chiprpy, dynamic_acceleration in zip(orient_sensor, accdyn_sensor):
            rotation_chip = eul2rotm(chiprpy)
            rotation = rotation_chip.dot(calib.imu_bases[imu])
            rotation_matrix_world = calib.imu_bases[imu].T.dot(rotation)
            eul = rotm2eul(rotation_matrix_world)
            orient_world = numpy.append(orient_world, [[eul[0]], [eul[1]], [eul[2]]], 1)

            rot = rotation.T.dot(dynamic_acceleration)
            accdyn_world = numpy.append(accdyn_world, [[rot[0]], [rot[1]], [rot[2]]], 1)

        return (accdyn_world, orient_world)



    # USING "pyquaternion"
    def get_orientation_madgwick(self, data, calib, imu, filter_num_samples, initwindow=0.5, beta=2.86):

        # GET ACC IN MPS2
        acc  = numpy.array(data.as_list_of_triples(imu, 'accel'))

        # GET GYRO IN RADIANS PER SEC
        gyro = numpy.array(data.as_list_of_triples(imu, 'gyro'))

        # FILTER DATA
        acc  = self.filter(acc, filter_num_samples)
        gyro = self.filter(gyro, filter_num_samples)

        # CONVERT ACCEL DATA TO GS
        acc = acc / 9.81

        # GET TIME IN MS
        #time = numpy.array(data.imu_data['timestamps'])
        # CONVERT TIME TO SECONDS
        #time = time / 1000.0

        # COMPUTE TIME ARRAY IN SECONDS, ASSUME GLOBAL VALUE IS CORRECT
        time = numpy.array(numpy.arange(0, data.num_samples*Ic_global.SECONDS_PER_SAMPLE, Ic_global.SECONDS_PER_SAMPLE))

        qchip2world = Quaternion(matrix=calib.imu_bases[imu])

        qorient = [0] * time.size

        sampfreq = 1.0/numpy.mean(numpy.diff(time))

        dt = 1.0 / sampfreq

        qorient[0] = qchip2world.conjugate


        for i, gyro1 in enumerate(gyro[1:, :], start=1):
            qprev = qorient[i-1]

            acc1 = acc[i, :]
            acc1 = acc1 / numpy.linalg.norm(acc1)

            # QUATERNION ANGULAR CHANGE FROM THE GRYO
            qdotgyro = 0.5 * (qprev * Quaternion(0, gyro1[0], gyro1[1], gyro1[2]))


            if beta > 0:
                # GRADIENT DESCENT ALGORITHM CORRECTIVE STEP
                qp = qprev.elements

                F = numpy.array([2*(qp[1]*qp[3] - qp[0]*qp[2]) - acc1[0],
                              2*(qp[0]*qp[1] + qp[2]*qp[3]) - acc1[1],
                              2*(0.5 - qp[1]**2 - qp[2]**2) - acc1[2]])
                J = numpy.array([[-2*qp[2], 2*qp[3], -2*qp[0], 2*qp[1]],
                               [2*qp[1], 2*qp[0], 2*qp[3], 2*qp[2]],
                               [0, -4*qp[1], -4*qp[2], 0]])

                step = numpy.dot(J.T, F)
                step = step / numpy.linalg.norm(step)

                step = Quaternion(*step)

                qdot = qdotgyro - (numpy.deg2rad(beta) * step)
            else:
                qdot = qdotgyro

            qorient[i] = qprev + (qdot * dt)

            qorient[i] = qorient[i].normalised


        # GET THE GRAVITY VECTOR
        # GRAVITY IS +Z

        gvec = [(q.conjugate * Quaternion(0, 0, 0, 1) * q).elements[1:] for q in qorient]

        accdyn_sensor = acc - gvec

        # ATTEMPT TO CONVERT FROM CHIP COORDINATES TO WORLD
        # qorient IS THE QUATERNION THAT SPECIFIES THE CURRENT ORIENTATION OF THE CHIP, RELATIVE TO ITS INITIAL ORIENTATION
        # qchip2world IS THE QUATERNION THAT ROTATES FROM THE INITIAL CHIP ORIENTATION TO THE WORLD FRAME


        qorient_world = [qchip2world.conjugate * q1.conjugate for q1 in qorient]


        orient_world_rotm = [q1.rotation_matrix for q1 in qorient_world]


        orient_world = [rotm2eul(R1) for R1 in orient_world_rotm]


        # ROTATE ACCDYN INTO THE WORLD COORDINATE SYSTEM

        qaccdyn_world = [(qchip2world.conjugate * Quaternion(0, a1[0], a1[1], a1[2]) * qchip2world) for a1 in accdyn_sensor]


        accdyn_world = [q.elements[1:] for q in qaccdyn_world]

        # CONVERT ACCEL DATA BACK TO MPS2
        accdyn_world = [i * 9.81 for i in accdyn_world]

        #return (accdyn_world, orient_world, orient_world_rotm)
        return (accdyn_world, orient_world)











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



# def gramschmidt(U):
#     k = U.shape[1]
#     V = copy(U)
# 
#     for i in range(k):
#         V[:, i] /= numpy.linalg.norm(V[:, i])
# 
#         for j in range(i+1, k):
#             proj = numpy.dot(V[:, i], V[:, j]) * V[:, i]
#             V[:, j] -= proj
# 
#     return V


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



