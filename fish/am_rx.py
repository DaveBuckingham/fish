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
    COM_PACKET_SAMPLE               = 0x50
    COM_PACKET_ASA                  = 0x51
    COM_PACKET_STRING               = 0x54
    COM_PACKET_TEST                 = 0x55
    COM_PACKET_HELLO                = 0x56
    COM_PACKET_NUMIMUS              = 0x57

    # SINGLE BYTE COMMANDS TO SEND FROM PC TO ARDUINO
    COM_SIGNAL_INIT                 = 0x69
    COM_SIGNAL_ASA                  = 0x61
    COM_SIGNAL_RUN                  = 0x72
    COM_SIGNAL_STOP                 = 0x73
    COM_SIGNAL_TEST                 = 0x74
    COM_SIGNAL_HELLO                = 0x68


    GYRO_SENSITIVITY             = 131     # if range is +- 250
    ACCEL_SENSITIVITY            = 16384   # if range is +- 2

    PLOT_REFRESH_RATE            = 20


    MAGNETOMETER_SCALE_FACTOR    = 0.15
    # WHO_AM_I                     = 0x71

    codes = {str(COM_PACKET_NUMIMUS): 'COM_PACKET_NUMIMUS', 
        str(COM_PACKET_SAMPLE): 'COM_PACKET_SAMPLE', 
        str(COM_PACKET_HELLO): 'COM_PACKET_HELLO', 
        str(COM_PACKET_ASA): 'COM_PACKET_ASA', 
        str(COM_SIGNAL_INIT): 'COM_SIGNAL_INIT', 
        str(COM_SIGNAL_ASA): 'COM_SIGNAL_ASA', 
        str(COM_SIGNAL_RUN): 'COM_SIGNAL_RUN', 
        str(COM_SIGNAL_STOP): 'COM_SIGNAL_STOP', 
        str(COM_SIGNAL_HELLO): 'COM_SIGNAL_HELLO', 
        }

    mag_asas = []


    finished_signal = pyqtSignal()
    message_signal = pyqtSignal(QString)
    error_signal = pyqtSignal(QString)
    numimus_signal = pyqtSignal(int)

    recording_signal = pyqtSignal()


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


        self.imu_data = {}
        self.imu_data['timestamps'] = []

        self.end_timestamp = 'inf'

        self.use_trigger = False

        self.data_lock = [False]
        


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
        #time.sleep(1)
        #print("WRITE: " + self.codes[str(val)])
        self.connection.write(bytearray((val,)))
        
    # SHOULD ONLY BE USED BY rx_packet()
    def rx_byte(self):
        val = self.connection.read(1)
        if (len(val) == 0):
            return None
        else:
            return struct.unpack("<B", val)[0]
            #return ord(val)


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
            if (val == Am_rx.COM_FLAG_ESCAPE):
                val = self.rx_byte()
                if (val == None):
                    return (None, None)
                val = val ^ Am_rx.COM_FLAG_XOR
            message.append(val)
            val = self.rx_byte()
            if (val == None):
                return (None, None)

        #print("READ: " + self.codes[str(message_type)])
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
            return False


        # HANDSHAKE
        self.tx_byte(Am_rx.COM_SIGNAL_STOP)
        time.sleep(2)
        self.connection.flushInput()
        self.tx_byte(Am_rx.COM_SIGNAL_HELLO)

        handshake_success = None
        handshake_attempts = 0

        while (handshake_success is None):
            self.message_signal.emit("attempting handshake\n")
            (message, message_type) = self.rx_packet()
            if ((message_type is not None) and (message_type == Am_rx.COM_PACKET_HELLO)):
                handshake_success = True
            else:
                handshake_attempts += 1

                if (handshake_attempts > 4):
                    handshake_success = False
                else:
                    self.tx_byte(Am_rx.COM_SIGNAL_STOP)
                    time.sleep(2)
                    self.connection.flushInput()
                    self.tx_byte(Am_rx.COM_SIGNAL_HELLO)

        if (handshake_success):
            self.message_signal.emit("handshake succesfull\n")
        else:
            self.error_signal.emit("handshake failed\n")
            self.close_connection()

        return handshake_success



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

    def abort(self):
        self.tx_byte(Am_rx.COM_SIGNAL_STOP)
        self.close_connection()
        self.finished_signal.emit()
        self.recording = False


    @pyqtSlot()
    def run(self):

        if (not self.open_connection()):
            self.error_signal.emit("No connection, aborting.\n")
            self.abort()

        self.message_signal.emit("initializing imus\n")
        self.tx_byte(Am_rx.COM_SIGNAL_INIT)
        time.sleep(1)
        (message, message_type) = self.rx_packet()
        if (message_type == Am_rx.COM_PACKET_NUMIMUS):
            self.num_imus = message[0];
            self.message_signal.emit("Detected " + str(self.num_imus) + " IMUs.\n")
        else:
            self.error_signal.emit("Unable to determine number of IMUs, aborting.\n")
            self.abort()


        if (self.num_imus < 1):
            self.error_signal.emit("No IMUs detected, aborting.\n")
            self.abort()

        self.sample_length = 4 + (18 * self.num_imus) + 1
        if (Am_rx.USE_ENCODER):
            self.sample_length += 2

        time.sleep(2)


        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        self.message_signal.emit("calculating magnetometer sensitivty adjustment...\n")
        Am_rx.mag_asas = []
        time.sleep(1)
        self.tx_byte(Am_rx.COM_SIGNAL_ASA)
        time.sleep(1)

        for i in (range(0, self.num_imus)):
            (received, message_type) = self.rx_packet()
            if ((message_type == Am_rx.COM_PACKET_ASA) and (len(received) == 3)):
                asa = []
                for j in (range(0, 3)):
                    asa.append(((float(received[j] - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR)
                Am_rx.mag_asas.append(asa)
            else:
                print(str(message_type))
                print(str(len(received)))
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

        self.recording_signal.emit()

        # RESET TIMER
        start_time = time.time() * 1000

        sample_index = 0

        # DICTIONARY OF LISTS AND OF LISTS OF DICTIONARIES OF LISTS OF LISTS
        self.data_lock[0] = True
        self.imu_data = {}
        self.imu_data['timestamps'] = []
        self.imu_data['imus'] = [{'accel': [[],[],[]], 'gyro': [[],[],[]], 'mag': [[],[],[]]}] * self.num_imus
        if (Am_rx.USE_ENCODER):
            self.imu_data['encoder'] = []
        self.data_lock[0] = False

        # MUST BE AFTER imu_data SET UP
        self.numimus_signal.emit(self.num_imus)

        while (self.recording):

            (received, message_type) = self.rx_packet()

            if ((Am_rx.USE_ENCODER and len(received) < (18*self.num_imus) + 7) or (len(received) < (18*self.num_imus) + 5)):
                self.error_signal.emit("Sample packet too short: " + str(len(received)) + "\n")


            if ((message_type == Am_rx.COM_PACKET_SAMPLE) and (len(received) == self.sample_length)):

                timestamp = (time.time() * 1000) - start_time


                received = bytearray(received)
               

                (id,) = struct.unpack('>L', received[:4])
                del received[:4]


                if (Am_rx.USE_ENCODER):
                    (enc,) = struct.unpack('>h', received[:2])
                    del received[:2]
                    enc *= 0.3515625  # 360/1024

                self.data_lock[0] = True

                self.imu_data['timestamps'].append(timestamp)
                for i in (range(0, self.num_imus)):
                    #print(ord(received))
                    (ax, ay, az, gx, gy, gz) = struct.unpack('>hhhhhh', received[:12])
                    del received[:12]
                    (mx, my, mz) = struct.unpack('<hhh', received[:6]) # BIG ENDIAN
                    del received[:6]
                    (gx, gy, gz, gx, gy, gz) = map(lambda x: float(x) / Am_rx.GYRO_SENSITIVITY, (gx, gy, gz, gx, gy, gz))
                    (mx, my, mz) = [(mx, my, mz)[j] * Am_rx.mag_asas[i][j] for j in range(3)]

                    #print(mx, my, mz)


                    self.imu_data['imus'][i]['accel'][0].append(ax)
                    self.imu_data['imus'][i]['accel'][1].append(ay)
                    self.imu_data['imus'][i]['accel'][2].append(az)

                    self.imu_data['imus'][i]['gyro'][0].append(gx)
                    self.imu_data['imus'][i]['gyro'][1].append(gy)
                    self.imu_data['imus'][i]['gyro'][2].append(gz)

                    self.imu_data['imus'][i]['mag'][0].append(mx)
                    self.imu_data['imus'][i]['mag'][1].append(my)
                    self.imu_data['imus'][i]['mag'][2].append(mz)

                (trig,) = struct.unpack('>?', str(received[0]))
                del received[0]


                if (Am_rx.USE_ENCODER):
                    self.imu_data['encoder'].append(enc)

                self.data_lock[0] = False


                if (len(received) > 0):
                    self.error_signal.emit("Sample packet too long.\n")



                if (trig and self.use_trigger):
                    self.message_signal.emit("received trigger\n")
                    self.abort()

                sample_index += 1

                #self.timestamp_signal.emit(timestamp)

                count = sample_index % Am_rx.PLOT_REFRESH_RATE

            else:
                print("unknown sample received. type: " + str(message_type) + ", len: " + str(len(received)))


        self.message_signal.emit("stopping recording data\n")
        self.abort()


