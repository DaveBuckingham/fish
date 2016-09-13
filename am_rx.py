#!/usr/bin/python

import os
import sys
import time
import serial
import struct
import random
import array
from PyQt4.QtCore import *

USE_ENCODER = False

class Am_rx(QObject):

    COM_FLAG                     = 0x7E
    COM_ESCAPE                   = 0X7D
    COM_XOR                      = 0X20
    COM_MAX_PACKET_LENGTH        = 500


    GYRO_SENSITIVITY             = 131     # if range is +- 250
    ACCEL_SENSITIVITY            = 16384   # if range is +- 2

    PLOT_REFRESH_RATE            = 50

    if (USE_ENCODER):
        DATA_LENGTH                  = 42
    else:
        DATA_LENGTH                  = 40

    MAGNETOMETER_SCALE_FACTOR    = 0.15
    WHO_AM_I                     = 0x71


    mag_0_asa = [1, 1, 1]
    mag_1_asa = [1, 1, 1]


    finished_signal = pyqtSignal()
    message_signal = pyqtSignal(QString)
    error_signal = pyqtSignal(QString)

    timestamp_signal = pyqtSignal(float)
    plot_a0_signal = pyqtSignal(float, list, bool)
    plot_a1_signal = pyqtSignal(float, list, bool)
    plot_g0_signal = pyqtSignal(float, list, bool)
    plot_g1_signal = pyqtSignal(float, list, bool)
    plot_m0_signal = pyqtSignal(float, list, bool)
    plot_m1_signal = pyqtSignal(float, list, bool)


    def __init__(self, parent = None):
        super(Am_rx, self).__init__(parent)

        self.recording = False

        self.data = []
        self.end_timestamp = 'inf'

        #self.use_trigger
        #self.pre_trigger
        #self.post_trigger



    def calculate_accel_ft(self, a_test):
        # SEE MPU-6000, 6050 MANUAL PAGE 11
        # DON'T NEGATE Y-AXIS VALUE
        if (a_test == 0):
            return 0
        else:
            exponent = (a_test - 1) / 30
            return (1392.64 * ((0.092 ** exponent) / 0.34))


    def calculate_gyro_ft(self, g_test):
        # SEE MPU-6000, 6050 MANUAL PAGE 10
        # FOR Y-AXIS, NEGATE RETURN VALUE
        if (g_test == 0):
            return 0
        else:
            return (3275 * (1.046 ** (g_test - 1)))


    def self_test(self):
        # GYRO RANGE SHOULD BE +=8g (MPU6050)
        # ACCEL RANGE SHOULD BE +=250dps (MPU6050)
        pass


    def cleanup(self):
        self.message_signal.emit("closing serial connection")
        self.connection.close()

    def tx_byte(self, val):
        self.connection.write(struct.pack('!c', val))
        
    def rx_byte(self):
        val = self.connection.read(1)
        if (len(val) == 0):
            return None
        else:
            return ord(val)


    # READ A PACKET FROM SERIAL AND RETURN ITS CONTENTS
    # IF TIMEOUT, RETURN AN EMPTY LIST
    def rx_packet(self):
        message = []
        val = None

        # WAIT FOR START OF FRAME
        while (val != Am_rx.COM_FLAG):
            val = self.rx_byte()
            if (val == None):
                return []

        # CHECK FOR CONTIGUOUS COM FLAGS, E.G. CAUGHT END OF FRAME
        while (val == Am_rx.COM_FLAG):
            val = self.rx_byte()
            if (val == None):
                return []

        # READ, UNSTUFF, AND STORE PAYLOAD
        while (val != Am_rx.COM_FLAG):
            # UNSTUFF
            if (val == Am_rx.COM_ESCAPE):
                val = self.rx_byte()
                if (val == None):
                    return []
                val = val ^ Am_rx.COM_XOR
            message.append(val)
            val = self.rx_byte()
            if (val == None):
                return []

        return message





    @pyqtSlot()
    def run(self):

        self.message_signal.emit("establishing serial connection... ")

        try:
            self.connection = serial.Serial(
                port     = '/dev/ttyACM0' if (os.name == 'posix') else 'COM1',
                baudrate = 115200,
                parity   = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
               #timeout  = None   # block, wait forever
               #timeout  = 0      # non-blocking, return immedietly up to number of requested bytes
                timeout  = 1.0    # 1 second timeout 
            )

        except serial.serialutil.SerialException:
            self.error_signal.emit("FAILED\n")
            self.finished_signal.emit()
            return

        self.message_signal.emit("OK\n")

        self.message_signal.emit("waiting for arduino to reset\n")
        time.sleep(3)

        self.message_signal.emit("initializing imus\n")
        self.tx_byte('i')


        time.sleep(3)


        ##################################
        #          WHO AM I ?????        #
        ##################################
        received = []
        # while ((len(received) == 0) or (received != [Am_rx.WHO_AM_I, Am_rx.WHO_AM_I])):

        self.message_signal.emit("reading imu whoamis... ")
        self.tx_byte('w')
        received = self.rx_packet()
        if (received == [Am_rx.WHO_AM_I, Am_rx.WHO_AM_I]):
            self.message_signal.emit("OK\n")
        elif (len(received) == 0):
            self.error_signal.emit("FAILED (timeout)\n")
        elif (received != [Am_rx.WHO_AM_I, Am_rx.WHO_AM_I]):
            self.error_signal.emit("FAILED (values read: " + ', '.join(map(str, received)) + ")\n")




        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        self.message_signal.emit("calculating magnetometer sensitivty adjustment... ")
        self.tx_byte('m')
        received = self.rx_packet()

        if (len(received) == 6):
            for i in (range(0, 3)):
                Am_rx.mag_0_asa[i] = ((float(received[i]     - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR
                Am_rx.mag_1_asa[i] = ((float(received[i + 3] - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR
            self.message_signal.emit("OK\n")
            self.message_signal.emit("mag_0 asa: " + ', '.join(map(str, Am_rx.mag_0_asa)) + "\n")
            self.message_signal.emit("mag_1 asa: " + ', '.join(map(str, Am_rx.mag_1_asa)) + "\n")

        else:
            self.error_signal.emit("FAILED\n")
            self.error_signal.emit("using 1 adjustment\n")




        self.tx_byte('r')
        self.message_signal.emit("waiting to stabilize\n")
        for i in range(0, 50):
            self.rx_packet()

        self.message_signal.emit("recording data\n")

        # RESET TIMER
        start_time = time.time() * 1000

        sample_index = 0
        self.data = []

        while (self.recording):

            received = self.rx_packet()
            if (len(received) == Am_rx.DATA_LENGTH):
                received = array.array('B', received).tostring()

#                 (id, enc, temp0, ax0, ay0, az0, gx0, gy0, gz0, magid0, status1_0, mx0, my0, mz0, status2_0,   \
#                           temp1, ax1, ay1, az1, gx1, gy1, gz1, magid1, status1_1, mx1, my1, mz1, status2_1) = \
#                     struct.unpack('>LhhhhhhhhBBhhhBhhhhhhhBBhhhB', received)

                if (USE_ENCODER):
                    (id, enc, ax0, ay0, az0, gx0, gy0, gz0, mx0, my0, mz0, ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, ) = struct.unpack('>Lhhhhhhhhhhhhhhhhhhh', received)
                    enc *= 0.3515625  # 360/1024
                else:
                    (id, ax0, ay0, az0, gx0, gy0, gz0, mx0, my0, mz0, ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, ) = struct.unpack('>Lhhhhhhhhhhhhhhhhhh', received)


                timestamp = (time.time() * 1000) - start_time


                # CONVERT
                (ax0, ay0, az0, ax1, ay1, az1) = map(lambda x: float(x) / Am_rx.ACCEL_SENSITIVITY, (ax0, ay0, az0, ax1, ay1, az1))
                (gx0, gy0, gz0, gx1, gy1, gz1) = map(lambda x: float(x) / Am_rx.GYRO_SENSITIVITY,  (gx0, gy0, gz0, gx1, gy1, gz1))
                (mx0, my0, mz0) = [(mx0, my0, mz0)[i] * Am_rx.mag_0_asa[i] for i in range(3)]
                (mx1, my1, mz1) = [(mx1, my1, mz1)[i] * Am_rx.mag_1_asa[i] for i in range(3)]

                # (temp0, temp1) = map(lambda x: (float(x) / 340.0) + 36.53, (temp0, temp1))
                # (temp0, temp1) = map(lambda x: (float(x) / 340.0) + 21.0, (temp0, temp1))
                # (temp0, temp1) = map(lambda x: (float(x) / 333.87) + 21.0, (temp0, temp1))




#                print("id {:d}   temp {:.2f}   status1 {:08b}   status2 {:08b} id {:d} temp {:.2f}   status1 {:08b}   status2 {:08b}".format(  magid0, temp0, status1_0, status2_0,  magid1, temp1, status1_1, status2_1))



                entry = {}
                entry['time']    = timestamp
                entry['accel0']  = [ax0, ay0, az0]
                entry['gyro0']   = [gx0, gy0, gz0]
                entry['mag0']    = [mx0, my0, mz0]
                entry['accel1']  = [ax1, ay1, az1]
                entry['gyro1']   = [gx1, gy1, gz1]
                entry['mag1']    = [mx1, my1, mz1]

                if (USE_ENCODER):
                    entry['encoder'] = enc


                self.data.append(entry)
                sample_index += 1

                self.timestamp_signal.emit(timestamp)

                #refresh = (sample_index % Am_rx.PLOT_REFRESH_RATE) == 0

                count = sample_index % Am_rx.PLOT_REFRESH_RATE

                self.plot_a0_signal.emit(timestamp, [ax0, ay0, az0], count == 0) 
                self.plot_a1_signal.emit(timestamp, [ax1, ay1, az1], count == 0) 
                self.plot_g0_signal.emit(timestamp, [gx0, gy0, gz0], count == 0) 
                self.plot_g1_signal.emit(timestamp, [gx1, gy1, gz1], count == 0) 
                self.plot_m0_signal.emit(timestamp, [mx0, my0, mz0], count == 0) 
                self.plot_m1_signal.emit(timestamp, [mx1, my1, mz1], count == 0) 

            else:
                print("packet received (length=" + str(len(received)) + "):"),
                print received



        self.tx_byte('s')

        # UNWRAP DATA SO IT CAN BE PROCESSED OR SAVED
        #self.data = self.data[sample_index:] + self.data[:sample_index]

        self.finished_signal.emit()

        self.connection.flushInput()
        self.connection.flushOutput()




