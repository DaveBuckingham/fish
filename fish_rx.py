#!/usr/bin/python

import os
import sys
import time
import serial
import struct
import h5py
import atexit



################################################
#          CONSTANTS AND VARIABLES             #
################################################


SENTINEL_1                 = 0xF0
SENTINEL_2                 = 0x0D
DATA_FILENAME              = 'sample.hdf5'
PRINT_TO_STDOUT            = True
GYRO_SENSITIVITY           = 131     # if range is +- 250
ACCEL_SENSITIVITY          = 16384   # if range is +- 2

sample_index = 0



################################################
#                  FUNCTIONS                   #
################################################


def message(string):
    print " ><>  " + string + "..."


def calculate_accel_ft(a_test):
    # SEE MPU-6000, 6050 MANUAL PAGE 11
    # DON'T NEGATE Y-AXIS VALUE
    if (a_test == 0):
        return 0
    else:
        exponent = (a_test - 1) / 30
        return (1392.64 * ((0.092 ** exponent) / 0.34))


def calculage_gyro_ft(g_test):
    # SEE MPU-6000, 6050 MANUAL PAGE 10
    # FOR Y-AXIS, NEGATE RETURN VALUE
    if (g_test == 0):
        return 0
    else:
        return (3275 * (1.046 ** (g_test - 1)))


def self_test():
    # GYRO RANGE SHOULD BE +=8g (MPU6050)
    # ACCEL RANGE SHOULD BE +=250dps (MPU6050)
    pass


def save():
    message("saving data to file '" + DATA_FILENAME + "'")
    datafile = h5py.File(DATA_FILENAME, 'w')
    data = datafile.create_group("data")
    data.create_dataset('t',      data=data_time)
    data.create_dataset('Accel',  data=data_accel1)
    data.create_dataset('Accel2', data=data_accel2)
    data.create_dataset('Gyro',   data=data_gyro1)
    data.create_dataset('Gyro2',  data=data_gyro2)
    datafile.close()


def cleanup():
    save()
    message("closing serial connection")
    connection.close()
    message("exiting")



################################################
#                 INITIALIZE                   #
################################################


# REGISTER CLEANUP FUNCTION
atexit.register(cleanup)

data_time   = []
data_accel1 = []
data_accel2 = []
data_gyro1  = []
data_gyro2  = []


# OPEN SERIAL CONNECTION
message("opening serial connection")
connection = serial.Serial(
    port     = '/dev/ttyACM0' if (os.name == 'posix') else 'COM1',
    baudrate = 115200,
    parity   = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout  = None   # block, wait forever
)


# WASTE THE FIRST FEW READINGS, WHICH ARE USUALLY CHAOTIC
message("waiting to stabilize")
connection.read(1000)


# RESET TIMER
start_time = time.time() * 1000
last_time = start_time



################################################
#           MAIN LOOP, RECORD DATA             #
################################################


message("recording data")
while (True):

    # WAIT TWO-BYTE SENTINEL, THEN READ DATA
    flag_0 = ord(connection.read(1))
    while (flag_0 != SENTINEL_1):
        flag_0 = ord(connection.read(1))

    if (ord(connection.read(1)) == SENTINEL_2):
        
        # READ FROM IMU SENSORS
        data = connection.read(30)
        #(id, enc, ax0, ay0, az0, gx0, gy0, gz0, ax1, ay1, az1, gx1, gy1, gz1) = struct.unpack('!LHHHHHHHHHHHHH', data)
        (id, enc, ax0, ay0, az0, gx0, gy0, gz0, ax1, ay1, az1, gx1, gy1, gz1) = struct.unpack('!Lhhhhhhhhhhhhh', data)
        timestamp = (time.time() * 1000) - start_time
        difference = timestamp - last_time

        # CONVERT
        (ax0, ay0, az0, ax1, ay1, az1) = map(lambda x: float(x) / ACCEL_SENSITIVITY, (ax0, ay0, az0, ax1, ay1, az1))
        (gx0, gy0, gz0, gx1, gy1, gz1) = map(lambda x: float(x) / GYRO_SENSITIVITY,  (gx0, gy0, gz0, gx1, gy1, gz1))
        enc *= 0.3515625  # 360/1024

        # WRITE TO DATA FILE
        data_time.append(timestamp)
        data_accel1.append([ax0, ay0, az0])
        data_accel2.append([ax1, ay1, az1])
        data_gyro1.append([gx0, gy0, gz0])
        data_gyro2.append([gx1, gy1, gz1])


        # PRINT DATA TO STDOUT
        if (PRINT_TO_STDOUT):
            sys.stdout.write("%-6d %-12.4f %-7.4f " % (id, timestamp, difference))
            sys.stdout.write("%f " % enc)
            sys.stdout.write("%f %f %f " % (ax0, ay0, az0))
            sys.stdout.write("%f %f %f " % (gx0, gy0, gz0))
            sys.stdout.write("%f %f %f " % (ax1, ay1, az1))
            sys.stdout.write("%f %f %f " % (gx1, gy1, gz1))
            sys.stdout.write("\n")

        # UPDATE
        last_time = timestamp
        sample_index += 1




