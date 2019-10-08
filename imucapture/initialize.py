import math
import sys
import os
import time
import serial
import struct
import logging
import serial.tools.list_ports

from imucapture.txrx import Txrx

from imucapture.global_data import Global_data

import PyQt5.QtCore

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Initialize(PyQt5.QtCore.QObject):

    mag_asas = []

    finished_signal = PyQt5.QtCore.pyqtSignal()
    success_signal = PyQt5.QtCore.pyqtSignal()
    numimus_signal = PyQt5.QtCore.pyqtSignal(int)
    asa_signal = PyQt5.QtCore.pyqtSignal(list)
    rate_signal = PyQt5.QtCore.pyqtSignal(int)


    def __init__(self):
        super(Initialize, self).__init__()


    def initialize(self):

        txrx = Txrx()

        if (not txrx.open_connection()):
            logging.warning("failed to create connection, aborting initialization")
            self.finished_signal.emit()
            return(False)

        txrx.initialize_arduino()




        ##################################
        #           NUM IMUS             #
        ##################################

        num_imus = 0

        logging.info("requesting number of imus from arduino...")
        (message, message_type) = txrx.rx_packet()
        if (message_type == Txrx.COM_PACKET_NUMIMUS):
            num_imus = message[0];
        else:
            logging.warning("failed to determine number of imus, aborting")
            txrx.close_connection()
            self.finished_signal.emit()
            return()

        if (num_imus < 1):
            logging.warning("found zero imus, aborting")
            txrx.close_connection()
            self.finished_signal.emit()
            return()

        logging.info("found " + str(num_imus) + " imu(s)");
        self.numimus_signal.emit(num_imus)

        time.sleep(2)


        ##################################
        #        MAG ADJUSTMENT          #
        ##################################

        logging.info("calculating magnetometer sensitivty adjustment...")
        mag_asas = []
        time.sleep(1)
        txrx.tx_byte(Txrx.COM_SIGNAL_ASA)
        time.sleep(1)

        for i in (range(0, num_imus)):
            (received, message_type) = txrx.rx_packet()
            if ((message_type == Txrx.COM_PACKET_ASA) and (len(received) == 3)):
                asa = []
                for j in (range(0, 3)):
                    asa.append((float(received[j] - 128.0) / 256.0) + 1.0)
                    #asa.append(((float(received[j] - 128) / 256) + 1) * Txrx.MAGNETOMETER_SCALE_FACTOR)
                mag_asas.append(asa)
            else:
                logging.warning("ASA read failed, using 1 adjustment")
                mag_asas.append([1,1,1])

        self.asa_signal.emit(mag_asas)



        ##################################
        #          SAMPLE RATE           #
        ##################################

        logging.info("requesting sample rate from arduino...")
        time.sleep(1)
        txrx.tx_byte(Txrx.COM_SIGNAL_RATE)
        time.sleep(1)

        (received, message_type) = txrx.rx_packet()
        if ((message_type == Txrx.COM_PACKET_RATE) and (len(received) == 2)):
            sample_rate = (received[0] << 8) | received[1]
            logging.info("sample rate received: " + str(sample_rate));


        else:
            logging.warning("sample rate read failed, aborting");
            txrx.close_connection()
            self.finished_signal.emit()
            return()

        self.rate_signal.emit(sample_rate)






        txrx.close_connection()

        self.success_signal.emit()
        self.finished_signal.emit()
        return






    # GYRO RANGE SHOULD BE +=8g (MPU6050)
    # ACCEL RANGE SHOULD BE +=250dps (MPU6050)
    def test(self):
        self.open_connection()
        self.tx_byte(Txrx.COM_SIGNAL_TEST)
        time.sleep(1)
        (received, message_type) = self.rx_packet()
        self.close_connection()

        if ((message_type == Txrx.COM_PACKET_TEST) and (len(received) == 6)):
            return [bool(val) for val in received]

        else:
            return None


