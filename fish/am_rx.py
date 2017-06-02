import os
import time
import serial
import struct
import logging
import serial.tools.list_ports

from fish.am_data import Am_data

import PyQt5.QtCore

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Am_rx(PyQt5.QtCore.QObject):

    USE_ENCODER = False

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


    finished_signal = PyQt5.QtCore.pyqtSignal()
    numimus_signal = PyQt5.QtCore.pyqtSignal(int)

    recording_signal = PyQt5.QtCore.pyqtSignal()

    trigger_state = None

    timestamp_signal = PyQt5.QtCore.pyqtSignal(float)

    def __init__(self, data, settings):
        super(Am_rx, self).__init__()

        self.recording = False

        self.sample_length = 0

        self.data = data
        self.settings = settings

        self.trigger_value = False




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
        self.message_signal.emit("closing serial connection")
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
                logging.warning("rx failed, no data read from serial")
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
        logging.info("establishing serial connection...")
        
        # FIND A PORT CONNECTED TO AN ARDUINO
        arduino_ports = [ p.device for p in serial.tools.list_ports.comports() if (p.manufacturer and ('Arduino' in p.manufacturer)) ]
        if not arduino_ports:
            logging.warning('no Arduino found')
            return False
        else:
            serial_port = arduino_ports[0]
            logging.info('using Arduino found on ' + serial_port)

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
            logging.info("serial connection established")

        except serial.serialutil.SerialException:
            return False


        # HANDSHAKE
        self.tx_byte(Am_rx.COM_SIGNAL_STOP)
        time.sleep(2)
        self.connection.flushInput()
        self.tx_byte(Am_rx.COM_SIGNAL_HELLO)

        handshake_success = None
        handshake_attempts = 0

        while (handshake_success is None):
            logging.info("attempting handshake")
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
            logging.info("handshake succesfull")
        else:
            logging.error("handshake failed")
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





    #@pyqtSlot()
    def run(self):

        if (not self.open_connection()):
            logging.warning("failed to create connection, aborting")
            self.finished_signal.emit()
            return()

        logging.info("initializing imus")
        self.tx_byte(Am_rx.COM_SIGNAL_INIT)
        time.sleep(1)
        (message, message_type) = self.rx_packet()
        if (message_type == Am_rx.COM_PACKET_NUMIMUS):
            num_imus = message[0];
            logging.info("detected " + str(num_imus) + " IMUs")
        else:
            logging.error("unable to determine number of IMUs, aborting")
            self.close_connection()
            self.finished_signal.emit()
            return()

        self.data.reset_data(num_imus)

        if (num_imus < 1):
            logging.warning("no IMUs detected, aborting")
            self.close_connection()
            self.finished_signal.emit()
            return()

        # MUST BE AFTER imu_data SET UP
        self.numimus_signal.emit(self.data.num_imus)

        self.sample_length = 4 + (18 * num_imus) + 1
        if (Am_rx.USE_ENCODER):
            self.sample_length += 2

        time.sleep(2)


        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        logging.info("calculating magnetometer sensitivty adjustment...")
        Am_rx.mag_asas = []
        time.sleep(1)
        self.tx_byte(Am_rx.COM_SIGNAL_ASA)
        time.sleep(1)

        for i in (range(0, num_imus)):
            (received, message_type) = self.rx_packet()
            if ((message_type == Am_rx.COM_PACKET_ASA) and (len(received) == 3)):
                asa = []
                for j in (range(0, 3)):
                    asa.append(((float(received[j] - 128) / 256) + 1) * Am_rx.MAGNETOMETER_SCALE_FACTOR)
                Am_rx.mag_asas.append(asa)
            else:
                print(str(message_type))
                print(str(len(received)))
                logging.warning("ASA read failed, using 1 adjustment")
                Am_rx.mag_asas.append([1,1,1])

        logging.info("Magnetometer asa values:")
        for imu in Am_rx.mag_asas:
            logging.info(imu)
            #logging.info(', '.join(map(str, imu)))


        ##################################
        #        RECORD DATA             #
        ##################################

        self.tx_byte(Am_rx.COM_SIGNAL_RUN)
        logging.info("sent record command to arduino")


        logging.info("recording data")

        self.recording_signal.emit()

        # RESET TIMER
        start_time = time.time() * 1000

        #sample_index = 0



        while (self.recording):

            (received, message_type) = self.rx_packet()


            if ((message_type == Am_rx.COM_PACKET_SAMPLE) and (len(received) == self.sample_length)):

                timestamp = (time.time() * 1000) - start_time


                received = bytearray(received)
               

                (id,) = struct.unpack('>L', received[:4])


                sample = []
                sample.append(timestamp)
                sample.append([])

                for i in (range(0, self.data.num_imus)):

                    (ax, ay, az, gx, gy, gz) = struct.unpack('>hhhhhh', received[4:16])
                    (mx, my, mz) = struct.unpack('<hhh', received[16:22]) # BIG ENDIAN
                    (gx, gy, gz, gx, gy, gz) = map(lambda x: float(x) / Am_rx.GYRO_SENSITIVITY, (gx, gy, gz, gx, gy, gz))
                    (mx, my, mz) = [(mx, my, mz)[j] * Am_rx.mag_asas[i][j] for j in range(3)]

                    sample[1].append([[ax, ay, az], [gx, gy, gz], [mx, my, mz]])

                (self.trigger_state,) = struct.unpack('>?', received[22:23])

                if (self.settings.invert_trigger):
                    self.trigger_state = not self.trigger_state

                if (Am_rx.USE_ENCODER):
                    (enc,) = struct.unpack('>h', received[23:25])
                    enc *= 0.3515625  # 360/1024
                    sample.append(enc)

                self.data.add_sample(sample, self.settings.data_buffer_len)


                if (self.settings.use_trigger and self.trigger_state):
                    self.message_signal.emit("received trigger")
                    self.tx_byte(Am_rx.COM_SIGNAL_STOP)
                    self.close_connection()
                    self.finished_signal.emit()
                    return()

            else:
                logging.error("unknown sample received. type: " + str(message_type) + ", len: " + str(len(received)))


        logging.info("stopping recording data")
        self.close_connection()
        self.finished_signal.emit()
        return()

