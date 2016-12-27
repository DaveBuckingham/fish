#!/usr/bin/python

import os
import sys
import time
import serial
import warnings
import struct
import random
import array

import serial.tools.list_ports

from PyQt4.QtCore import pyqtSignal, QObject, pyqtSlot

try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = str


class Am_rx(QObject):

    USE_ENCODER = True

    COM_FLAG_START                  = 0x7E
    COM_FLAG_END                    = 0x7F
    COM_FLAG_ESCAPE                 = 0x7D
    COM_FLAG_XOR                    = 0x20

    # TO SPECIFY TYPE OF A (POSSIBLY EMPTY) PACKET SENT FROM ARDUINO TO PC
    COM_PACKET_SAMPLE               = 0x60     # 96
    COM_PACKET_ASA                  = 0x61     # 97
    COM_PACKET_STRING               = 0x64
    COM_PACKET_TEST                 = 0x65
    COM_PACKET_HELLO                = 0x66

    # SINGLE BYTE COMMANDS TO SEND FROM PC TO ARDUINO
    COM_SIGNAL_INIT                 = 0x50
    COM_SIGNAL_ASA                  = 0x52
    COM_SIGNAL_RUN                  = 0x53
    COM_SIGNAL_STOP                 = 0x54
    COM_SIGNAL_TEST                 = 0x55
    COM_SIGNAL_HELLO                = 0x56






    GYRO_SENSITIVITY             = 131     # if range is +- 250
    ACCEL_SENSITIVITY            = 16384   # if range is +- 2

    PLOT_REFRESH_RATE            = 20

    if (USE_ENCODER):
        DATA_LENGTH              = 43
    else:
        DATA_LENGTH              = 41

    MAGNETOMETER_SCALE_FACTOR    = 0.15
    # WHO_AM_I                     = 0x71


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

        self.use_trigger = False


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




    def close_connection(self):
        self.message_signal.emit("closing serial connection\n")
        self.connection.flushInput()
        self.connection.close()

    # CAN PROBABLY GET RID OF THIS FUNCTION
    def tx_byte(self, val):
        #self.connection.write(struct.pack('!c', val))
        #self.connection.write(bytes(val))
        self.connection.write(chr(val))
        
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
        while (val != Am_rx.COM_FLAG_START):
            val = self.rx_byte()
            if (val == None):
                self.error_signal.emit("rx failed, no data read from serial\n")
                return (None, None)

        message_type = self.rx_byte()
        val = self.rx_byte()

        # READ, UNSTUFF, AND STORE PAYLOAD
        while (val != Am_rx.COM_FLAG_END):
            # UNSTUFF
            if (val == Am_rx.COM_FLAG_ESCAPE):
                val = self.rx_byte()
                if (val == None):
                    return (None, None)
                val = val ^ Am_rx.COM_FLAG_XOR
            message.append(val)
            val = self.rx_byte()
            if (val == None):
                return (None, None)

        return (message, message_type)


    def open_connection(self):
        self.message_signal.emit("establishing serial connection...\n")



        
        arduino_ports = [ p.device for p in serial.tools.list_ports.comports() if (p.manufacturer and ('Arduino' in p.manufacturer)) ]
        if not arduino_ports:
            self.error_signal.emit('No arduino found\n')
            return False
        else:
            serial_port = arduino_ports[0]
            self.message_signal.emit('Using Arduino found on ' + serial_port + "\n")

        try:
            self.connection = serial.Serial(
                #port     = '/dev/ttyACM0' if (os.name == 'posix') else 'COM1',
                port     = serial_port,
                baudrate = 115200,
                parity   = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
               #timeout  = None   # block, wait forever
               #timeout  = 0      # non-blocking, return immedietly up to number of requested bytes
                timeout  = 1.0    # 1 second timeout 
            )

        except serial.serialutil.SerialException:
            self.error_signal.emit("failed to create connection\n")
            # self.finished_signal.emit()
            return False

        self.connection.flushInput()
        time.sleep(3)
        self.tx_byte(Am_rx.COM_SIGNAL_HELLO)
        time.sleep(1)
        (message, message_type) = self.rx_packet()
        if ((message_type is not None) and (message_type == Am_rx.COM_PACKET_HELLO)):
            self.message_signal.emit("serial connection established, handshake succesfull\n")
            return True
        else:
            self.error_signal.emit("serial connection established but handshake failed\n")
            self.close_connection()
            return False








    # GYRO RANGE SHOULD BE +=8g (MPU6050)
    # ACCEL RANGE SHOULD BE +=250dps (MPU6050)
    def test(self):
        self.open_connection()
        self.tx_byte(Am_rx.COM_SIGNAL_TEST)
        time.sleep(1)
        (received, message_type) = self.rx_packet()
        self.close_connection()

        if ((message_type == Am_rx.COM_PACKET_TEST) and (len(received) == 6)):
            return [bool(val) for val in received]

        else:
            return None

            




    @pyqtSlot()
    def run(self):

        if (not self.open_connection()):
            self.finished_signal.emit()
            return

        self.message_signal.emit("initializing imus\n")
        self.tx_byte(Am_rx.COM_SIGNAL_INIT)

        time.sleep(3)



        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        self.message_signal.emit("calculating magnetometer sensitivty adjustment... ")
        self.tx_byte(Am_rx.COM_SIGNAL_ASA)
        (received, message_type) = self.rx_packet()
        print(received)

        if ((message_type == Am_rx.COM_PACKET_ASA) and (len(received) == 6)):
            for i in (range(0, 3)):
                Am_rx.mag_0_asa[i] = ((float(received[i]     - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR
                Am_rx.mag_1_asa[i] = ((float(received[i + 3] - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR
            self.message_signal.emit("OK\n")
            self.message_signal.emit("mag_0 asa: " + ', '.join(map(str, Am_rx.mag_0_asa)) + "\n")
            self.message_signal.emit("mag_1 asa: " + ', '.join(map(str, Am_rx.mag_1_asa)) + "\n")

        else:
            self.error_signal.emit("FAILED\n")
            self.error_signal.emit("using 1 adjustment\n")


        self.message_signal.emit("sent record command to arduino\n")



        self.tx_byte(Am_rx.COM_SIGNAL_RUN)


        self.message_signal.emit("recording data\n")

        # RESET TIMER
        start_time = time.time() * 1000

        sample_index = 0
        self.data = []

        while (self.recording):

            (received, message_type) = self.rx_packet()
            if ((message_type == Am_rx.COM_PACKET_SAMPLE) and (len(received) == Am_rx.DATA_LENGTH)):
                received = array.array('B', received).tostring()


                if (Am_rx.USE_ENCODER):
                    (id, enc, ax0, ay0, az0, gx0, gy0, gz0, mx0, my0, mz0, ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, trig ) = struct.unpack('>Lhhhhhhhhhhhhhhhhhhh?', received)
                    enc *= 0.3515625  # 360/1024
                else:
                    (id, ax0, ay0, az0, gx0, gy0, gz0, mx0, my0, mz0, ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, trig ) = struct.unpack('>Lhhhhhhhhhhhhhhhhhh?', received)

                # SWAP MAG BYTES SO VALUES ARE BIG-ENDIAN
                (mx0, my0, mz0, mx1, my1, mz1) = struct.unpack('<hhhhhh', struct.pack('>hhhhhh', mx0, my0, mz0, mx1, my1, mz1))



                if (trig and self.use_trigger):
                    self.message_signal.emit("received trigger\n")
                    self.recording = False
                    break

                timestamp = (time.time() * 1000) - start_time


                # CONVERT
                (ax0, ay0, az0, ax1, ay1, az1) = map(lambda x: float(x) / Am_rx.ACCEL_SENSITIVITY, (ax0, ay0, az0, ax1, ay1, az1))
                (gx0, gy0, gz0, gx1, gy1, gz1) = map(lambda x: float(x) / Am_rx.GYRO_SENSITIVITY,  (gx0, gy0, gz0, gx1, gy1, gz1))
                (mx0, my0, mz0) = [(mx0, my0, mz0)[i] * Am_rx.mag_0_asa[i] for i in range(3)]
                (mx1, my1, mz1) = [(mx1, my1, mz1)[i] * Am_rx.mag_1_asa[i] for i in range(3)]

                # (temp0, temp1) = map(lambda x: (float(x) / 340.0) + 36.53, (temp0, temp1))
                # (temp0, temp1) = map(lambda x: (float(x) / 340.0) + 21.0, (temp0, temp1))
                # (temp0, temp1) = map(lambda x: (float(x) / 333.87) + 21.0, (temp0, temp1))



                entry = {}
                entry['time']    = timestamp
                entry['accel0']  = [ax0, ay0, az0]
                entry['gyro0']   = [gx0, gy0, gz0]
                entry['mag0']    = [mx0, my0, mz0]
                entry['accel1']  = [ax1, ay1, az1]
                entry['gyro1']   = [gx1, gy1, gz1]
                entry['mag1']    = [mx1, my1, mz1]

                if (Am_rx.USE_ENCODER):
                    entry['encoder'] = enc


                self.data.append(entry)
                sample_index += 1

                self.timestamp_signal.emit(timestamp)

                count = sample_index % Am_rx.PLOT_REFRESH_RATE

                self.plot_a0_signal.emit(timestamp, [ax0, ay0, az0], count == 0) 
                self.plot_a1_signal.emit(timestamp, [ax1, ay1, az1], count == 0) 
                self.plot_g0_signal.emit(timestamp, [gx0, gy0, gz0], count == 0) 
                self.plot_g1_signal.emit(timestamp, [gx1, gy1, gz1], count == 0) 
                self.plot_m0_signal.emit(timestamp, [mx0, my0, mz0], count == 0) 
                self.plot_m1_signal.emit(timestamp, [mx1, my1, mz1], count == 0) 

            else:
                print("unknown sample received. type: " + str(message_type))


        self.message_signal.emit("stopping recording data\n")
        self.tx_byte(Am_rx.COM_SIGNAL_STOP)



        self.close_connection()

        self.finished_signal.emit()



