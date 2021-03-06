import math
import sys
import os
import time
import serial
import struct
import logging
import serial.tools.list_ports

from imucapture.data import Data

from imucapture.global_data import Global_data

class Txrx():


    COM_FLAG_START                  = 0x7E
    COM_FLAG_END                    = 0x7F
    COM_FLAG_ESCAPE                 = 0x7D
    COM_FLAG_XOR                    = 0x20

    # TO SPECIFY TYPE OF A (POSSIBLY EMPTY) PACKET SENT FROM ARDUINO TO PC
    COM_PACKET_SAMPLE               = 0x50
    COM_PACKET_ASA                  = 0x51
    COM_PACKET_TEST                 = 0x53
    COM_PACKET_HELLO                = 0x54
    COM_PACKET_NUMIMUS              = 0x55
    COM_PACKET_ERROR                = 0x56

    # SINGLE BYTE COMMANDS TO SEND FROM PC TO ARDUINO
    COM_SIGNAL_INIT                 = 0x60
    COM_SIGNAL_ASA                  = 0x61
    COM_SIGNAL_RUN                  = 0x63
    COM_SIGNAL_STOP                 = 0x64
    COM_SIGNAL_TEST                 = 0x65
    COM_SIGNAL_HELLO                = 0x66

    MAGNETOMETER_SCALE_FACTOR       = 0.15

    INT_MAX                         = (2**16)/2

    ACCEL_RANGE                     = 2       # +-. GS. SET BY ARDUINO CODE
    GYRO_RANGE                      = 250     # +-. DEGREES PER SECOND. SET BY ARDUINO CODE
    MAG_RANGE                       = 4800    # +-. MICRO TESLAS. CAN'T BE SET. MAYBE 4900?

    NO_TRIGGER_EDGE                 = 0x01
    RISING_TRIGGER_EDGE             = 0x02
    FALLING_TRIGGER_EDGE            = 0x03

    # USED FOR DEBUGGING
    codes = {str(COM_PACKET_NUMIMUS): 'COM_PACKET_NUMIMUS', 
             str(COM_PACKET_SAMPLE):  'COM_PACKET_SAMPLE', 
             str(COM_PACKET_HELLO):   'COM_PACKET_HELLO', 
             str(COM_PACKET_ASA):     'COM_PACKET_ASA', 
             str(COM_PACKET_ERROR):   'COM_PACKET_ERROR', 
             str(COM_SIGNAL_INIT):    'COM_SIGNAL_INIT', 
             str(COM_SIGNAL_ASA):     'COM_SIGNAL_ASA', 
             str(COM_SIGNAL_RUN):     'COM_SIGNAL_RUN', 
             str(COM_SIGNAL_STOP):    'COM_SIGNAL_STOP', 
             str(COM_SIGNAL_HELLO):   'COM_SIGNAL_HELLO', 
             }


    def __init__(self):
        pass

    def initialize_arduino(self):
        self.tx_byte(self.COM_SIGNAL_INIT)
        time.sleep(1)




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
        while (val != Txrx.COM_FLAG_START):
            val = self.rx_byte()
            if (val == None):
                logging.warning("rx failed, no data read from serial")
                return (None, None)

        message_type = self.rx_byte()
        val = self.rx_byte()

        # READ, UNSTUFF, AND STORE PAYLOAD
        while (val != Txrx.COM_FLAG_END):
            if (val == Txrx.COM_FLAG_ESCAPE):
                val = self.rx_byte()
                if (val == None):
                    return (None, None)
                val = val ^ Txrx.COM_FLAG_XOR
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
            logging.info(str(len(arduino_ports)) + ' possible Arduino device(s) found')
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
        self.tx_byte(Txrx.COM_SIGNAL_STOP)
        time.sleep(2)
        self.connection.flushInput()
        self.tx_byte(Txrx.COM_SIGNAL_HELLO)

        handshake_success = None
        handshake_attempts = 0

        while (handshake_success is None):
            logging.info("attempting handshake")
            (message, message_type) = self.rx_packet()
            if ((message_type is not None) and (message_type == Txrx.COM_PACKET_HELLO)):
                handshake_success = True
            else:
                handshake_attempts += 1

                if (handshake_attempts > 4):
                    handshake_success = False
                else:
                    self.tx_byte(Txrx.COM_SIGNAL_STOP)
                    time.sleep(2)
                    self.connection.flushInput()
                    self.tx_byte(Txrx.COM_SIGNAL_HELLO)

        if (handshake_success):
            logging.info("handshake succesfull")
        else:
            logging.error("handshake failed")
            self.close_connection()

        return handshake_success



