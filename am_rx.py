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

    finished_signal = pyqtSignal()
    sample_signal = pyqtSignal(list)
    message_signal = pyqtSignal(QString)
    error_signal = pyqtSignal(QString)

    def __init__(self, parent = None):
        super(Am_rx, self).__init__(parent)

        self.sample_index = 0
        self.recording = False


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
        connection.close()


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


        self.message_signal.emit("begin recording data")



        # MAKE FAKE DATA
        while (self.recording):
            self.sample_signal.emit( [(10*random.random() - 5) for i in xrange(14)] )
            time.sleep(.1)

        # COLLECT REAL DATA



        self.message_signal.emit("stop recording data")
        self.finished_signal.emit()




