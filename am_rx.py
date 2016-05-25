#!/usr/bin/python

import os
import sys
import time
import serial
import struct
import random
from PyQt4.QtCore import *

class Am_rx(QObject):

    SENTINEL_1                 = 0xF0
    SENTINEL_2                 = 0x0D
    GYRO_SENSITIVITY           = 131     # if range is +- 250
    ACCEL_SENSITIVITY          = 16384   # if range is +- 2

    PLOT_REFRESH_RATE          = 5

    finished_signal = pyqtSignal()
    message_signal = pyqtSignal(QString)
    error_signal = pyqtSignal(QString)

    timestamp_signal = pyqtSignal(float)
    plot_a1_signal = pyqtSignal(list, bool)
    plot_a2_signal = pyqtSignal(list, bool)
    plot_g1_signal = pyqtSignal(list, bool)
    plot_g2_signal = pyqtSignal(list, bool)

    def __init__(self, parent = None):
        super(Am_rx, self).__init__(parent)

        self.recording = False

        self.data = {'time':[], 'accel1':[], 'accel2':[], 'gyro1':[], 'gyro2':[]}
        self.num_samples = 0


    def calculate_accel_ft(self, a_test):
        # SEE MPU-6000, 6050 MANUAL PAGE 11
        # DON'T NEGATE Y-AXIS VALUE
        if (a_test == 0):
            return 0
        else:
            exponent = (a_test - 1) / 30
            return (1392.64 * ((0.092 ** exponent) / 0.34))


    def calculage_gyro_ft(self, g_test):
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


    @pyqtSlot()
    def run_fake(self):
        self.message_signal.emit("fake connection established")
        self.message_signal.emit("waiting to stabilize")
        self.message_signal.emit("begin recording data")
        start_time = time.time() * 1000

        while (self.recording):
            data = [(10*random.random() - 5) for i in xrange(14)] 
            timestamp = (time.time() * 1000) -  start_time

            self.data['time'].append(timestamp)
            self.data['accel1'].append(data[2:5]  )
            self.data['gyro1'].append( data[5:8]  )
            self.data['accel2'].append(data[8:11] )
            self.data['gyro2'].append( data[11:14])

            refresh = (self.num_samples % Am_rx.PLOT_REFRESH_RATE) == 0
            self.timestamp_signal.emit(timestamp)
            self.plot_a1_signal.emit(data[2:5], refresh)
            self.plot_g1_signal.emit(data[5:8], refresh)
            self.plot_a2_signal.emit(data[8:11], refresh)
            self.plot_g2_signal.emit(data[11:14], refresh)

            self.num_samples += 1

            time.sleep(.005)

        self.message_signal.emit("stop recording data")
        self.finished_signal.emit()


    @pyqtSlot()
    def run(self):

        try:
            self.connection = serial.Serial(
                port     = '/dev/ttyACM0' if (os.name == 'posix') else 'COM1',
                baudrate = 115200,
                parity   = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
                timeout  = None   # block, wait forever
            )

        except serial.serialutil.SerialException:
            self.error_signal.emit("serial connection failed")
            self.finished_signal.emit()
            return


        self.message_signal.emit("serial connection established")
        self.message_signal.emit("waiting to stabilize")
        self.connection.read(1000)
        self.message_signal.emit("begin recording data")

        # RESET TIMER
        start_time = time.time() * 1000


        print "recording"
        while (self.recording):

            # WAIT TWO-BYTE SENTINEL, THEN READ DATA
            flag = ord(self.connection.read(1))
            while (flag != Am_rx.SENTINEL_1):
                flag = ord(self.connection.read(1))

            if (ord(self.connection.read(1)) == Am_rx.SENTINEL_2):
                
                # READ FROM IMU SENSORS
                data = self.connection.read(30)
                (id, enc, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2) = struct.unpack('!Lhhhhhhhhhhhhh', data)
                timestamp = (time.time() * 1000) - start_time

                # CONVERT
                (ax1, ay1, az1, ax2, ay2, az2) = map(lambda x: float(x) / Am_rx.ACCEL_SENSITIVITY, (ax1, ay1, az1, ax2, ay2, az2))
                (gx1, gy1, gz1, gx2, gy2, gz2) = map(lambda x: float(x) / Am_rx.GYRO_SENSITIVITY,  (gx1, gy1, gz1, gx2, gy2, gz2))
                enc *= 0.3515625  # 360/1024

                print enc

                self.data['time'].append(timestamp)
                self.data['accel1'].append( [ax1, ay1, az1])
                self.data['gyro1'].append(  [gx1, gy1, gz1])
                self.data['accel2'].append( [ax2, ay2, az2])
                self.data['gyro2'].append(  [gx2, gy2, gz2])

                refresh = (self.num_samples % Am_rx.PLOT_REFRESH_RATE) == 0
                self.timestamp_signal.emit(timestamp)
                self.plot_a1_signal.emit([ax1, ay1, az1], refresh)
                self.plot_g1_signal.emit([gx1, gy1, gz1], refresh)
                self.plot_a2_signal.emit([ax2, ay2, az2], refresh)
                self.plot_g2_signal.emit([gx2, gy2, gz2], refresh)


            # UPDATE
            self.num_samples += 1




        self.message_signal.emit("stop recording data")
        self.finished_signal.emit()




