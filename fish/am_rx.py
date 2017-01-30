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
    COM_PACKET_NUMIMUS              = 0x67

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


    MAGNETOMETER_SCALE_FACTOR    = 0.15
    # WHO_AM_I                     = 0x71


    # mag_0_asa = [1, 1, 1]
    # mag_1_asa = [1, 1, 1]
    mag_asas = []


    finished_signal = pyqtSignal()
    message_signal = pyqtSignal(QString)
    error_signal = pyqtSignal(QString)
    numimus_signal = pyqtSignal(int)


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

        self.sample_length = 0

        self.data = []
        self.end_timestamp = 'inf'

        self.use_trigger = False
        
        self.num_imus = 0


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

    # FOR SENDING COMMAND SIGNALS TO ARDUINO
    def tx_byte(self, val):
        #print(hex(val))
        self.connection.write(bytes(val))
        
    # SHOULD ONLY BE USED BY rx_packet()
    def rx_byte(self):
        val = self.connection.read(1)
        if (len(val) == 0):
            return None
        else:
            print((ord(val)))
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
        
        # FIND A PORT CONNECTED TO AN ARDUINO
        arduino_ports = [ p.device for p in serial.tools.list_ports.comports() if (p.manufacturer and ('Arduino' in p.manufacturer)) ]
        if not arduino_ports:
            self.error_signal.emit('No Arduino found\n')
            return False
        else:
            serial_port = arduino_ports[0]
            self.message_signal.emit('Using Arduino found on ' + serial_port + "\n")

        # CONNECT
        try:
            self.connection = serial.Serial(
                # port     = '/dev/ttyACM0' if (os.name == 'posix') else 'COM1',
                port     = serial_port,
                baudrate = 115200,
                parity   = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
                # timeout  = None   # block, wait forever
                # timeout  = 0      # non-blocking, return immedietly up to number of requested bytes
                timeout  = 1.0    # 1 second timeout 
            )
            self.message_signal.emit("serial connection established\n")

        except serial.serialutil.SerialException:
            self.error_signal.emit("failed to create connection\n")
            # self.finished_signal.emit()
            return False

        # GIVE ARDUINO TIME TO RESET
        self.connection.flushInput()
        time.sleep(3)

        # HANDSHAKE
        self.tx_byte(Am_rx.COM_SIGNAL_HELLO)
        time.sleep(1)
        (message, message_type) = self.rx_packet()
        if ((message_type is not None) and (message_type == Am_rx.COM_PACKET_HELLO)):
            self.message_signal.emit("handshake succesfull\n")
            return True
        else:
            self.error_signal.emit("handshake failed. received: " + str(int(message_type)) + "\n")
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
        (message, message_type) = self.rx_packet()
        if (message_type == Am_rx.COM_PACKET_NUMIMUS):
            self.num_imus = message;
            self.message_signal.emit("Detected " + str(self.num_imus) + " IMUs.\n")
        else:
            self.error_signal.emit("Unable to determine number of IMUs.\n")
            self.finished_signal.emit()
            return

        self.numimus_signal.emit(num_imus)

        self.sample_length = 4 + (12 * self.num_imus) + 1
        if (Am_rx.USE_ENCODER):
            self.sample_length += 2

        time.sleep(2)


        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        self.message_signal.emit("calculating magnetometer sensitivty adjustment... ")
        self.tx_byte(Am_rx.COM_SIGNAL_ASA)

        # if ((message_type == Am_rx.COM_PACKET_ASA) and (len(received) == 6)):
        #     for i in (range(0, 3)):
        #         Am_rx.mag_0_asa[i] = ((float(received[i]     - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR
        #         Am_rx.mag_1_asa[i] = ((float(received[i + 3] - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR
        #     self.message_signal.emit("OK\n")
        #     self.message_signal.emit("mag_0 asa: " + ', '.join(map(str, Am_rx.mag_0_asa)) + "\n")
        #     self.message_signal.emit("mag_1 asa: " + ', '.join(map(str, Am_rx.mag_1_asa)) + "\n")

        for i in (range(0, num_imus)):
            (received, message_type) = self.rx_packet()
            if ((message_type == Am_rx.COM_PACKET_ASA) and (len(received) == 3)):
                asa = []
                for j in (range(0, 3)):
                    asa.append(((float(received[j] - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR)
                Am_rx.mag_asas.append(asa)
            else:
                self.error_signal.emit("ASA read failed, using 1 adjustment\n")
                Am_rx.mag_asas.append([1,1,1])

        self.message_signal.emit("Magnetometer asa values:\n")
        for imu in Am_rx.mag_asas:
            self.message_signal.emit(', '.join(map(str, imu)) + "\n")


        ##################################
        #        RECORD DATA             #
        ##################################

        self.tx_byte(Am_rx.COM_SIGNAL_RUN)
        self.message_signal.emit("sent record command to arduino\n")


        self.message_signal.emit("recording data\n")

        # RESET TIMER
        start_time = time.time() * 1000

        sample_index = 0

        self.imu_data = {}
        self.imu_data['timestamps'] = []
        self.imu_data['imus'] = [[]] * num_imus
        if (Am_rx.USE_ENCODER):
            self.imu_data['encoder'] = []

        while (self.recording):

            (received, message_type) = self.rx_packet()
            if ((message_type == Am_rx.COM_PACKET_SAMPLE) and (len(received) == Am_rx.sample_length)):

                timestamp = (time.time() * 1000) - start_time

                self.imu_data['timestamps'].append(timestamp)

                received = bytearray(received)

                (id) = struct.unpack('>L', received[:4])
                del received[:4]

                if (Am_rx.USE_ENCODER):
                    (enc) = struct.unpack('>h', received[:2])
                    del received[:2]
                    enc *= 0.3515625  # 360/1024

                for i in (range(0, num_imus)):
                    (ax, ay, az, gx, gy, gz) = struct.unpack('>hhhhhh', received[:12])
                    del received[:12]
                    (mx, my, mz) = struct.unpack('<hhh', received[:6]) # BIG ENDIAN
                    del received[:6]
                    (gx, gy, gz, gx, gy, gz) = map(lambda x: float(x) / Am_rx.GYRO_SENSITIVITY, (gx, gy, gz, gx, gy, gz))
                    (mx, my, mz) = [(mx, my, mz)[i] * Am_rx.mag_0_asa[i] for i in range(3)]
                    imu_data['imus'][i]['accel'].append([ax, ay, az])
                    imu_data['imus'][i]['gyro'].append([gx, gy, gz])
                    imu_data['imus'][i]['mag'].append([mx, my, mz])

                (trig) = struct.unpack('>?', received[0])
                del received[0]

                if (len(received > 0)):
                    self.error_signal.emit("Sample packet too long.\n")

                if (Am_rx.USE_ENCODER):
                    imu_data['encoder'].append(enc)

                if (trig and self.use_trigger):
                    self.message_signal.emit("received trigger\n")
                    self.recording = False
                    break

                sample_index += 1

                self.timestamp_signal.emit(timestamp)

                count = sample_index % Am_rx.PLOT_REFRESH_RATE

                #self.plot_a0_signal.emit(timestamp, [ax0, ay0, az0], count == 0) 
                #self.plot_a1_signal.emit(timestamp, [ax1, ay1, az1], count == 0) 
                #self.plot_g0_signal.emit(timestamp, [gx0, gy0, gz0], count == 0) 
                #self.plot_g1_signal.emit(timestamp, [gx1, gy1, gz1], count == 0) 
                #self.plot_m0_signal.emit(timestamp, [mx0, my0, mz0], count == 0) 
                #self.plot_m1_signal.emit(timestamp, [mx1, my1, mz1], count == 0) 

            else:
                print("unknown sample received. type: " + str(message_type))


        self.message_signal.emit("stopping recording data\n")
        self.tx_byte(Am_rx.COM_SIGNAL_STOP)


        self.close_connection()

        self.finished_signal.emit()


