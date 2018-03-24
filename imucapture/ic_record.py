import math
import sys
import os
import time
import serial
import struct
import logging
import serial.tools.list_ports

from imucapture.ic_data import Ic_data
from imucapture.ic_rx import Ic_rx

from imucapture.ic_global import Ic_global

import PyQt5.QtCore

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_record(PyQt5.QtCore.QObject):


    finished_signal = PyQt5.QtCore.pyqtSignal()

    def __init__(self, settings, num_imus, mag_asas):
        super(Ic_record, self).__init__()

        # CAN WE MOVE THIS INTO record()?
        self.recording = True

        self.num_imus = num_imus

        self.sample_length = 4 + (18 * self.num_imus) + 1
        #if (Ic_global.USE_ENCODER):
        #    self.sample_length += 2

        self.settings = settings

        self.trigger_state = False


    def record(self):

        txrx = Ic_rx()

        txrx.open_connection()

        txrx.tx_byte(Ic_rx.COM_SIGNAL_RUN)
        logging.info("sent record command to arduino")

        # RESET TIMER
        start_time = time.time() * 1000

        while (self.recording):

            (received, message_type) = txrx.rx_packet()

            if ((message_type == Ic_rx.COM_PACKET_SAMPLE) and (len(received) == self.sample_length)):


                received = bytearray(received)

                (id,) = struct.unpack('>L', received[:4])

                sample = []

                for i in (range(0, self.num_imus)):


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

                    sample.append([[ax, ay, az], [gx, gy, gz], [mx, my, mz]])



                trigger_start = 4 + (self.num_imus * 18)

                # ONE BYTE FOR TRIGGER
                (self.trigger_state,) = struct.unpack('>?', received[trigger_start:trigger_start+1])

                if (not self.settings.rising_edge):
                    self.trigger_state = not self.trigger_state

                #if (Ic_global.USE_ENCODER):
                #    # TWO BYTES FOR ENCODER
                #    (enc,) = struct.unpack('>h', received[trigger_start+1:trigger_start+3])
                #    enc *= 0.3515625  # 360/1024
                #    sample.append(enc)

                self.data.add_sample(sample)


                if (self.settings.use_trigger and self.trigger_state):
                    logging.info("received trigger")
                    self.tx_byte(Ic_rx.COM_SIGNAL_STOP)
                    txrx.close_connection()
                    self.finished_signal.emit()
                    return()

            #else:
                #logging.error("unknown sample received. type: " + str(message_type) + ", len: " + str(len(received)))


        logging.info("stopping recording data")
        txrx.close_connection()
        self.finished_signal.emit()
        return()

