#!/usr/bin/python

import os
import sys
import time
import serial
import struct
from PyQt4.QtCore import *

class Am_rx(QObject):

    SENTINEL_1                 = 0xF0
    SENTINEL_2                 = 0x0D
    GYRO_SENSITIVITY           = 131     # if range is +- 250
    ACCEL_SENSITIVITY          = 16384   # if range is +- 2

    def __init__(self, parent = None):
        super(Am_rx, self).__init__(parent)

        self.sample_index = 0
        self.record = True
        self.count = 100
        self.finished_trigger = pyqtSignal()

#        self.connection = serial.Serial(
#            port     = '/dev/ttyACM0' if (os.name == 'posix') else 'COM1',
#            baudrate = 115200,
#            parity   = serial.PARITY_NONE,
#            stopbits = serial.STOPBITS_ONE,
#            bytesize = serial.EIGHTBITS,
#            timeout  = None   # block, wait forever
#        )



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
        message("closing serial connection")
        connection.close()


    @pyqtSlot()
    def run(self):
        print("recording data")
        while (self.count > 0):
        #while (self.record):
            self.count -= 1
            print "sample"
        print "done"
        self.finished_trigger.emit()
        print "DONE"




