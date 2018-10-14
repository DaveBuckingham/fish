import math
import sys
import os
import time
import serial
import struct
import logging
import serial.tools.list_ports

from imucapture.data import Data
from imucapture.txrx import Txrx

from imucapture.global_data import Global_data

import PyQt5.QtCore

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Record(PyQt5.QtCore.QObject):


    finished_signal = PyQt5.QtCore.pyqtSignal()

    def __init__(self, settings, data, mag_asas):
        super(Record, self).__init__()

        # CAN WE MOVE THIS INTO record()?
        self.recording = True

        self.data = data

        self.mag_asas = mag_asas

        self.sample_length = 4 + (18 * self.data.num_imus) + 1
        #if (Global_data.USE_ENCODER):
        #    self.sample_length += 2

        self.settings = settings

        self.trigger_edge = Txrx.NO_TRIGGER_EDGE
        self.trigger_timeout_counter = -1

    def raw_accel_to_meters_per_second_squared(self, raw):
        assert((raw >= -Txrx.INT_MAX) and (raw < Txrx.INT_MAX))
        gs = Txrx.ACCEL_RANGE * (raw / Txrx.INT_MAX)
        mps = gs * 9.80665
        return mps

    def raw_gyro_to_radians_per_second(self, raw):
        assert((raw >= -Txrx.INT_MAX) and (raw < Txrx.INT_MAX))
        dps = Txrx.GYRO_RANGE * (raw / Txrx.INT_MAX)
        rps = dps * (math.pi / 180.0)
        return rps

    def raw_mag_to_microteslas(self, raw):
        return raw * 0.15
        # assert((raw >= -Txrx.INT_MAX) and (raw < Txrx.INT_MAX))
        # mt = Txrx.MAG_RANGE * (raw / Txrx.INT_MAX)
        # return mt




    def record(self):

        txrx = Txrx()

        if (not txrx.open_connection()):
            logging.warning("failed to create connection, aborting recording")
            self.finished_signal.emit()
            return(False)

        txrx.initialize_arduino()

        txrx.tx_byte(Txrx.COM_SIGNAL_RUN)
        logging.info("sent record command to arduino")

        # RESET TIMER
        start_time = time.time() * 1000

        while (self.recording):

            # HALT IF TRIGGER TIMEOUT ELAPSED
            if (self.trigger_timeout_counter == 0):
                logging.info("trigger timeout elapsed")
                txrx.tx_byte(Txrx.COM_SIGNAL_STOP)
                txrx.close_connection()
                self.finished_signal.emit()
                return()

            # DECREMENT TRIGGER TIMEOUT COUNTER
            if (self.trigger_timeout_counter >= 0):
                self.trigger_timeout_counter -= 1

            (received, message_type) = txrx.rx_packet()

            if ((message_type == Txrx.COM_PACKET_SAMPLE) and (len(received) == self.sample_length)):


                received = bytearray(received)

                (id,) = struct.unpack('>L', received[:4])

                sample = []

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

                    # I'M NOT SURE WHICH OF THESE SHOULD BE FIRST
                    # ASA FIRST, THEN SCALE BY 0.15 SEEMS TO PRODUCE APPROPRIATE VALUES
                    # I.E. BETWEEN 25 AND 65 MICROTESLAS AT EARTHS SURFACE ACCORDING TO WIKIPEDIA
                    (mx, my, mz) = [(mx, my, mz)[j] * self.mag_asas[i][j] for j in range(3)]
                    (mx, my, mz) = map(self.raw_mag_to_microteslas, (mx, my, mz))

                    sample.append([[ax, ay, az], [gx, gy, gz], [mx, my, mz]])



                trigger_start = 4 + (self.data.num_imus * 18)

                # ONE BYTE FOR TRIGGER
                (self.trigger_edge,) = struct.unpack('>?', received[trigger_start:trigger_start+1])

                #if (not self.settings.rising_edge):
                #    self.trigger_state = not self.trigger_state

                #if (Global_data.USE_ENCODER):
                #    # TWO BYTES FOR ENCODER
                #    (enc,) = struct.unpack('>h', received[trigger_start+1:trigger_start+3])
                #    enc *= 0.3515625  # 360/1024
                #    sample.append(enc)

                self.data.add_sample(sample)


                if ((self.settings.use_trigger != Txrx.NO_TRIGGER_EDGE) and (self.settings.use_trigger == self.trigger_edge)):
                    logging.info("received trigger")
                    self.trigger_timeout_counter = self.settings.trigger_delay


            #else:
                #logging.error("unknown sample received. type: " + str(message_type) + ", len: " + str(len(received)))


        logging.info("stopping recording data")
        txrx.close_connection()
        self.finished_signal.emit()
        return()

