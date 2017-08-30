import math
import os
import time
import serial
import struct
import logging
import serial.tools.list_ports

from fish.ic_data import Ic_data

import PyQt5.QtCore

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_rx(PyQt5.QtCore.QObject):

    USE_ENCODER = False
    #USE_ENCODER = True

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

    MAGNETOMETER_SCALE_FACTOR       = 0.15

    INT_MAX                         = (2**16)/2

    ACCEL_RANGE                     = 2       # +-. GS. SET BY ARDUINO CODE
    GYRO_RANGE                      = 250     # +-. DEGREES PER SECOND. SET BY ARDUINO CODE
    MAG_RANGE                       = 4800    # +-. MICRO TESLAS. CAN'T BE SET. MAYBE 4900?

    # USED FOR DEBUGGING
    codes = {str(COM_PACKET_NUMIMUS): 'COM_PACKET_NUMIMUS', 
             str(COM_PACKET_SAMPLE):  'COM_PACKET_SAMPLE', 
             str(COM_PACKET_HELLO):   'COM_PACKET_HELLO', 
             str(COM_PACKET_ASA):     'COM_PACKET_ASA', 
             str(COM_SIGNAL_INIT):    'COM_SIGNAL_INIT', 
             str(COM_SIGNAL_ASA):     'COM_SIGNAL_ASA', 
             str(COM_SIGNAL_RUN):     'COM_SIGNAL_RUN', 
             str(COM_SIGNAL_STOP):    'COM_SIGNAL_STOP', 
             str(COM_SIGNAL_HELLO):   'COM_SIGNAL_HELLO', 
             }

    mag_asas = []


    finished_signal = PyQt5.QtCore.pyqtSignal()
    numimus_signal = PyQt5.QtCore.pyqtSignal(int)

    recording_signal = PyQt5.QtCore.pyqtSignal()

    trigger_state = None

    timestamp_signal = PyQt5.QtCore.pyqtSignal(float)

    def __init__(self, data, settings):
        super(Ic_rx, self).__init__()

        self.recording = False

        self.sample_length = 0

        self.data = data
        self.settings = settings

        self.trigger_value = False


    def raw_accel_to_meters_per_second_squared(self, raw):
        assert((raw >= -Ic_rx.INT_MAX) and (raw < Ic_rx.INT_MAX))
        gs = Ic_rx.ACCEL_RANGE * (raw / Ic_rx.INT_MAX)
        mps = gs * 9.80665
        return mps

    def raw_gyro_to_radians_per_second(self, raw):
        assert((raw >= -Ic_rx.INT_MAX) and (raw < Ic_rx.INT_MAX))
        dps = Ic_rx.GYRO_RANGE * (raw / Ic_rx.INT_MAX)
        rps = dps * (math.pi / 180.0)
        return rps

    def raw_mag_to_microteslas(self, raw):
        return raw * 0.15
        # assert((raw >= -Ic_rx.INT_MAX) and (raw < Ic_rx.INT_MAX))
        # mt = Ic_rx.MAG_RANGE * (raw / Ic_rx.INT_MAX)
        # return mt



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
        logging.info("closing serial connection")
        self.connection.flushInput()
        self.connection.close()


    # FOR SENDING COMMAND SIGNALS TO ARDUINO
    def tx_byte(self, val):
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
        while (val != Ic_rx.COM_FLAG_START):
            val = self.rx_byte()
            if (val == None):
                logging.warning("rx failed, no data read from serial")
                return (None, None)

        message_type = self.rx_byte()
        val = self.rx_byte()

        # READ, UNSTUFF, AND STORE PAYLOAD
        while (val != Ic_rx.COM_FLAG_END):
            if (val == Ic_rx.COM_FLAG_ESCAPE):
                val = self.rx_byte()
                if (val == None):
                    return (None, None)
                val = val ^ Ic_rx.COM_FLAG_XOR
            message.append(val)
            val = self.rx_byte()
            if (val == None):
                return (None, None)

        #print("READ: " + self.codes[str(message_type)])
        return (message, message_type)


    def open_connection(self):
        logging.info("establishing serial connection...")
        
        # FIND A PORT CONNECTED TO AN ARDUINO
        arduino_ports = [ p.device for p in serial.tools.list_ports.comports() if (p.manufacturer and ('Arduino' in p.manufacturer or 'Microsoft' in p.manufacturer)) ]
        # MY ARDUINO UNO COMES UP AS 'Arduino' ON MY LINUX BOX BUT AS 'Microsoft' ON WINDOWS
        # THIS SEEMS WEIRD SINCE THE PYSERIAL DOCUMENTATION SAYS: "USB manufacturer string, as reported by device."
        # SO WHY DOES THE STRING REPORTED BY THE DEVICE VARY ACCORDING TO WHICH OS IT IS REPORTING TO?

        if not arduino_ports:
            logging.warning('no Arduino found')
            return False
        else:
            logging.info(str(len(arduino_ports)) + ' possible Arduino devices(s) found')
            serial_port = arduino_ports[0]
            logging.info('using device found on ' + serial_port)

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
                timeout  = 1.0      # 1 second timeout 
            )
            logging.info("serial connection established")

        except serial.serialutil.SerialException:
            return False


        # HANDSHAKE
        self.tx_byte(Ic_rx.COM_SIGNAL_STOP)
        time.sleep(2)
        self.connection.flushInput()
        self.tx_byte(Ic_rx.COM_SIGNAL_HELLO)

        handshake_success = None
        handshake_attempts = 0

        while (handshake_success is None):
            logging.info("attempting handshake")
            (message, message_type) = self.rx_packet()
            if ((message_type is not None) and (message_type == Ic_rx.COM_PACKET_HELLO)):
                handshake_success = True
            else:
                handshake_attempts += 1

                if (handshake_attempts > 4):
                    handshake_success = False
                else:
                    self.tx_byte(Ic_rx.COM_SIGNAL_STOP)
                    time.sleep(2)
                    self.connection.flushInput()
                    self.tx_byte(Ic_rx.COM_SIGNAL_HELLO)

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
        self.tx_byte(Ic_rx.COM_SIGNAL_TEST)
        time.sleep(1)
        (received, message_type) = self.rx_packet()
        self.close_connection()

        if ((message_type == Ic_rx.COM_PACKET_TEST) and (len(received) == 6)):
            return [bool(val) for val in received]

        else:
            return None



    def run(self):

        if (not self.open_connection()):
            logging.warning("failed to create connection, aborting")
            self.finished_signal.emit()
            return()

        logging.info("initializing imus")
        self.tx_byte(Ic_rx.COM_SIGNAL_INIT)
        time.sleep(1)
        (message, message_type) = self.rx_packet()
        if (message_type == Ic_rx.COM_PACKET_NUMIMUS):
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
        if (Ic_rx.USE_ENCODER):
            self.sample_length += 2

        time.sleep(2)


        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        logging.info("calculating magnetometer sensitivty adjustment...")
        Ic_rx.mag_asas = []
        time.sleep(1)
        self.tx_byte(Ic_rx.COM_SIGNAL_ASA)
        time.sleep(1)

        for i in (range(0, num_imus)):
            (received, message_type) = self.rx_packet()
            if ((message_type == Ic_rx.COM_PACKET_ASA) and (len(received) == 3)):
                asa = []
                for j in (range(0, 3)):
                    asa.append((float(received[j] - 128.0) / 256.0) + 1.0)
                    #asa.append(((float(received[j] - 128) / 256) + 1) * Ic_rx.MAGNETOMETER_SCALE_FACTOR)
                Ic_rx.mag_asas.append(asa)
            else:
                print(str(message_type))
                print(str(len(received)))
                logging.warning("ASA read failed, using 1 adjustment")
                Ic_rx.mag_asas.append([1,1,1])

        logging.info("Magnetometer asa values:")
        for imu in Ic_rx.mag_asas:
            logging.info(imu)
            #logging.info(', '.join(map(str, imu)))


        ##################################
        #        RECORD DATA             #
        ##################################

        self.tx_byte(Ic_rx.COM_SIGNAL_RUN)
        logging.info("sent record command to arduino")


        logging.info("recording data")

        self.recording_signal.emit()

        # RESET TIMER
        start_time = time.time() * 1000

        timestamp = 0
        while (self.recording):

            (received, message_type) = self.rx_packet()

            if ((message_type == Ic_rx.COM_PACKET_SAMPLE) and (len(received) == self.sample_length)):

                #timestamp = (time.time() * 1000) - start_time


                received = bytearray(received)
               

                (id,) = struct.unpack('>L', received[:4])


                sample = []
                sample.append(timestamp)
                sample.append([])

                for i in (range(0, self.data.num_imus)):


                    accel_start = 4 + (i * 18)
                    mag_start   = 16 + (i * 18)
                    mag_end     = 22 + (i * 18)

                    # ACCEL AND GYRO ARE LITTLE ENDIAN
                    (ax, ay, az, gx, gy, gz) = struct.unpack('>hhhhhh', received[accel_start:mag_start])

                    # MAG IS BIG ENDIAN
                    (mx, my, mz) = struct.unpack('<hhh', received[mag_start:mag_end])

                    # CONVERT RAW MEASUREMENTS TO OUR FAVORITE UNITS
                    (ax, ay, az) = map(self.raw_accel_to_meters_per_second_squared, (ax, ay, az))
                    (gx, gy, gz) = map(self.raw_gyro_to_radians_per_second, (gx, gy, gz))


                    # I'M NOT SURE WHICH IF THESE SHOULD BE FIRST
                    # ASA FIRST, THEN SCALE BY 0.15 SEEMS TO PRODUCE APPROPRIATE VALUES
                    # I.E. BETWEEN 25 AND 65 MICROTESLAS AT EARTHS SURFACE ACCORDING TO WIKIPEDIA
                    (mx, my, mz) = [(mx, my, mz)[j] * Ic_rx.mag_asas[i][j] for j in range(3)]
                    (mx, my, mz) = map(self.raw_mag_to_microteslas, (mx, my, mz))


                    sample[1].append([[ax, ay, az], [gx, gy, gz], [mx, my, mz]])



                trigger_start = 4 + (self.data.num_imus * 18)

                # ONE BYTE FOR TRIGGER
                (self.trigger_state,) = struct.unpack('>?', received[trigger_start:trigger_start+1])

                if (self.settings.invert_trigger):
                    self.trigger_state = not self.trigger_state

                if (Ic_rx.USE_ENCODER):
                    # TWO BYTES FOR ENCODER
                    (enc,) = struct.unpack('>h', received[trigger_start+1:trigger_start+3])
                    enc *= 0.3515625  # 360/1024
                    print(enc)
                    sample.append(enc)

                self.data.add_sample(sample, self.settings.data_buffer_len)


                if (self.settings.use_trigger and self.trigger_state):
                    logging.info("received trigger")
                    self.tx_byte(Ic_rx.COM_SIGNAL_STOP)
                    self.close_connection()
                    self.finished_signal.emit()
                    return()

            else:
                logging.error("unknown sample received. type: " + str(message_type) + ", len: " + str(len(received)))

            timestamp += 5


        logging.info("stopping recording data")
        self.close_connection()
        self.finished_signal.emit()
        return()

