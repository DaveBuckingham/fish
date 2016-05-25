#!/usr/bin/env python

# USE PYTHON 2.6 OR LATER

import sys
import h5py



in_filename = sys.argv[1]
ref_filename = sys.argv[2]
out_filename = in_filename + "_out"

f = h5py.File(ref_filename, 'r')
times  = f['data'].get('t').value
index = len(times)
f.close


f = h5py.File(in_filename, 'r')
times  = f['data'].get('t').value
accel1 = f['data'].get('Accel').value
accel2 = f['data'].get('Accel2').value
gyro1  = f['data'].get('Gyro').value
gyro2  = f['data'].get('Gyro2').value
f.close()

times = times[index:]
accel1 = accel1[index:]
accel2 = accel2[index:]
gyro1 = gyro1[index:]
gyro2 = gyro2[index:]


f = h5py.File(out_filename, 'w')
save_data = f.create_group("data")
save_data.create_dataset('t',       data=times)
save_data.create_dataset('Accel',  data=accel1)
save_data.create_dataset('Accel2', data=accel2)
save_data.create_dataset('Gyro',   data=gyro1)
save_data.create_dataset('Gyro2',  data=gyro2)
f.close()

