#!/usr/bin/env python

# USE PYTHON 2.6 OR LATER

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from fish.am_rx import *
from fish.am_plot import *
from fish.am_settings import *
#from fish.am_process import *
from collections import namedtuple
import time
import h5py
import csv
import signal
import atexit
from collections import deque

class Am_gui(QWidget):

    # DISPLAYED IN WINDOW HEADER AND SUCH
    APPLICATION_NAME = 'IMU Collect 0.01'

    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    def __init__(self, parent = None):
        super(Am_gui, self).__init__(parent)


        #self.timer = QTimer();
        #QtCore.QTimer.connect(self.timer, QtCore.SIGNAL("timeout()"), self, QtCore.SLOT("update()"))

        ########################
        #       VARIABLES      #
        ########################

        # HAS THE COLLECTED DATA BEEN SAVED TO FILE?
        self.data_saved = True

        # CURRENTLY RECORDING DATA?
        self.recording = False

        # IF FALSE, RECORD BUTTON TO START AND STOP RECORDING, IGNORE PIN
        # IF TRUE, SECOND CLICK CAPTURES DATA IN RANGE FROM PRE TO POST DELAY, HALT ON PIN
        self.pre_trigger_delay = 3
        self.post_trigger_delay = 3

        # ALL THE BUTTONS IN THE MAIN WINDOW
        self.buttons = {}


        # TIMESTAMPS OF COLLECTED DATA.
        #self.timestamps   = []

        #self.num_imus = 0

        # NUMBER OF SAMPLES COLLECTED
        self.num_samples = 0

        # MEASURED FREQUENCY OF DATA COLLECTED
        self.true_frequency = 0.0

        self.last_data_path = ''

        # HOLD ALL VISUAL ELEMENTS IN GUI MAIN WINDOW
        top_layout = QGridLayout()


        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)




        ##################################################
        #   SET UP SEPARATE THREAD FOR RECEIVING DATA    #
        ##################################################

        self.receiver_thread = QThread()
        self.receiver = Am_rx()
        self.receiver.moveToThread(self.receiver_thread)






        ##################################################
        #   CREATE GUI ELEMENTS AND ADD TO MAIN WINDOW   #
        ##################################################
        

        # CREATE BUTTONS AND ADD TO BUTTON LAYOUT

        button_layout = QVBoxLayout()
        self.button_container = QWidget()
        self.button_container.setLayout(button_layout)

        self.buttons['record'] = QPushButton('Record')
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])


        # self.buttons['test'] = QPushButton('Test')
        # self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        # self.buttons['test'].clicked.connect(self.test_button_slot)
        # button_layout.addWidget(self.buttons['test'])

        self.buttons['save'] = QPushButton('Save')
        self.buttons['save'].setToolTip('Save recorded data')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['load'] = QPushButton('Load')
        self.buttons['load'].setToolTip('Load recorded data')
        self.buttons['load'].clicked.connect(self.load_button_slot)
        button_layout.addWidget(self.buttons['load'])

        self.buttons['quit'] = QPushButton('Quit')
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        button_layout.addWidget(self.buttons['quit'])


        # TEXT OUTPUT WINDOW

        self.text_window = QTextEdit()
        self.text_window.setReadOnly(True)
        #print self.text_window.minimumHeight()
        self.text_window.setMinimumHeight(150)



        # SETTINGS (AFTER RECEIVER, NEEDS ACCESS TO use_trigger)

        self.settings = Am_settings(self)


        # STATUS INFO

        stats_layout = QGridLayout()
        self.stats_num_samples = QLabel("Samples:")
        self.stats_true_frequency = QLabel("Frequency:")
        self.stats_time = QLabel("Time:")

        stats_layout.addWidget(self.stats_num_samples, 1, 2)
        stats_layout.addWidget(self.stats_true_frequency, 1, 3)
        stats_layout.addWidget(self.stats_time, 1, 4)
        stats_layout.setColumnMinimumWidth(2, 120)

        self.plots_layout = QGridLayout()

        # ADD WIDGETS TO LAYOUT
        top_layout.addLayout(self.plots_layout, 1, 1)

        top_layout.addWidget(self.text_window, 4, 1, 1, 2)
        top_layout.addLayout(stats_layout, 5, 1, 1, 2)

        top_layout.addWidget(self.button_container, 1, 4, 3, 1)
        top_layout.addWidget(self.settings, 4, 4)


        label = QtGui.QLabel("Accel.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 1)

        label = QtGui.QLabel("Gyro.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 2)

        label = QtGui.QLabel("Mag.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 3)


        # ADD TOP LEVEL LAYOUT
        self.setLayout(top_layout)


        self.buttons['record'].setFocus()


        # SET WINDOW TITLE
        self.setWindowTitle(Am_gui.APPLICATION_NAME) 




        ########################
        #    QT CONNECTIONS    #
        ########################



        self.receiver.finished_signal.connect(self.receiver_thread.quit)

        self.receiver.recording_signal.connect(self.start_plot_slot)

        # USE TO TEST WITHOUT ARDUINO, RANDOMLY GENERATED DATA
        # self.receiver_thread.started.connect(self.receiver.run_fake)

        # COLLECT DATA FROM Am_rx() i.e. from arduino
        self.receiver_thread.started.connect(self.receiver.run)

        self.receiver_thread.finished.connect(self.receiver_done)




        self.receiver.message_signal.connect(self.message_slot)
        self.receiver.error_signal.connect(self.error_slot)
        self.receiver.numimus_signal.connect(self.numimus_slot)



    def print_pass_fail(self, val):
        if (val):
            self.message_slot("PASS\n")
        else:
            self.error_slot("FAIL\n")


    def make_plots(self):
        self.plots = []
        for i in (range(0, self.receiver.num_imus)):
            plot_a = Am_plot(self.receiver.imu_data['imus'][i]['accel'], self.receiver.data_lock, self)
            plot_g = Am_plot(self.receiver.imu_data['imus'][i]['gyro'], self.receiver.data_lock, self)
            plot_m = Am_plot(self.receiver.imu_data['imus'][i]['mag'], self.receiver.data_lock, self)
            self.plots.append(plot_a)
            self.plots.append(plot_g)
            self.plots.append(plot_m)
            self.plots_layout.addWidget(plot_a, i+1, 1)
            self.plots_layout.addWidget(plot_g, i+1, 2)
            self.plots_layout.addWidget(plot_m, i+1, 3)

            label = QtGui.QLabel("IMU " + str(i+1))
            label.setAlignment(QtCore.Qt.AlignBottom | QtCore.Qt.AlignCenter)
            self.plots_layout.addWidget(label, i+1, 0)








############################################
#              BUTTON SLOTS                #
############################################


#    def test_button_slot(self):
#
#        results = self.receiver.test()
#
#        if (not results):
#            self.error_slot("Arduino com error.\n")
#
#        else:
#
#            self.message_slot("imu1 communication test...")
#            self.print_pass_fail(results[0])
#
#            self.message_slot("imu2 communication test...")
#            self.print_pass_fail(results[1])
#
#            self.message_slot("imu1 self test...")
#            self.message_slot("not implemented\n")
#
#            self.message_slot("imu2 self test...")
#            self.message_slot("not implemented\n")
#
#            self.message_slot("mag1 self test...")
#            self.message_slot("not implemented\n")
#
#            self.message_slot("mag2 self test...")
#            self.message_slot("not implemented\n")



    def quit_button_slot(self):
        result = (QMessageBox.question(self,
                                       Am_gui.APPLICATION_NAME,
                                       'Really quit?',
                                       QMessageBox.Yes | QMessageBox.No,
                                       QMessageBox.Yes))

        if (result == QMessageBox.Yes):
            self.stop_recording()
            time.sleep(.2)  # let thread finish
            self.message_slot("exiting")
            self.close()


    # CAN THIS BE SIMPLIFIED BY SETTING STOP RECORDING TIME IN AM_RX INSTEAD!
    def record_button_slot(self):
        if (self.recording):
            self.stop_recording()
        else:
            if (not self.data_saved):
                result = (QMessageBox.question(self,
                                               'Message',
                                               'Overwrite recorded data without saving?',
                                               QMessageBox.Yes | QMessageBox.No,
                                               QMessageBox.No))

            if (self.data_saved or (result == QMessageBox.Yes)):
                self.num_samples = 0
                self.record()


    def save_button_slot(self):

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        filename, filetype = QFileDialog.getSaveFileName(self, "Save data", self.last_data_path, "*.hdf5;;*.csv", options=options)

        if filename:
            filename = str(filename)
            self.last_data_path = os.path.dirname(filename)

            #if ( (len(filename) < 5) or (filename[-5:].lower() != '.hdf5') ):
            #    filename += '.hdf5'

            if (filetype == "*.hdf5"):
                with h5py.File(str(filename), 'w') as datafile:
                    save_data = datafile.create_group("data")

                    save_data.create_dataset('t', data=self.receiver.imu_data['timestamps'])
                    for i in range(0, len(self.receiver.imu_data['imus'])):
                        imu = self.receiver.imu_data['imus'][i]
                        extension = "" if i < 1 else str(i + 1)

                        save_data.create_dataset('Accel' + extension, data=zip(*self.receiver.imu_data['imus'][i]['accel']))
                        save_data.create_dataset('Gyro'  + extension, data=zip(*self.receiver.imu_data['imus'][i]['gyro']))
                        save_data.create_dataset('Mag'   + extension, data=zip(*self.receiver.imu_data['imus'][i]['mag']))

                    if (self.receiver.USE_ENCODER):
                        save_data.create_dataset('Encoder', data=self.receiver.imu_data['encoder'])

            elif (filetype == "*.csv"):
                with open(filename, 'wb') as datafile:
                    writer = csv.writer(datafile, delimiter=',')

                    for i in range(0, len(self.receiver.imu_data['timestamps'])):
                        row = [self.receiver.imu_data['timestamps'][i]]
                        for j in range(0, len(self.receiver.imu_data['imus'])):
                            row.append(self.receiver.imu_data['imus'][j]['accel'][0][i])
                            row.append(self.receiver.imu_data['imus'][j]['accel'][1][i])
                            row.append(self.receiver.imu_data['imus'][j]['accel'][2][i])
                            row.append(self.receiver.imu_data['imus'][j]['gyro'][0][i])
                            row.append(self.receiver.imu_data['imus'][j]['gyro'][1][i])
                            row.append(self.receiver.imu_data['imus'][j]['gyro'][2][i])
                            row.append(self.receiver.imu_data['imus'][j]['mag'][0][i])
                            row.append(self.receiver.imu_data['imus'][j]['mag'][1][i])
                            row.append(self.receiver.imu_data['imus'][j]['mag'][2][i])

                        if (self.receiver.USE_ENCODER):
                            row.append(self.receiver.imu_data['encoder'][i])
                        writer.writerow(row)

            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return

            self.message_slot("data saved to  " + filename + "\n")
            self.data_saved = True


    def load_button_slot(self):

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        filename, filetype = QFileDialog.getOpenFileName(self, "Choose a file", self.last_data_path, "*.hdf5;;*.csv", options=options)

        if filename:
            filename = str(filename)
            self.last_data_path = os.path.dirname(filename)

            if (filetype == "*.hdf5"):
                with h5py.File(filename, 'r') as datafile:
                    self.receiver.imu_data = {}
                    self.receiver.imu_data['timestamps'] = datafile.get('data/t')[()]
                    self.receiver.imu_data['imus'] = []

                    i = 0
                    ext = ""
                    while ('data/Accel' + ext in datafile and 'data/Gyro' + ext in datafile and 'data/Mag' + ext in datafile):
                        self.receiver.imu_data['imus'].append({})
                        self.receiver.imu_data['imus'][i]['accel'] = map(list, zip(*datafile.get('data/Accel' + ext)[()]))
                        self.receiver.imu_data['imus'][i]['gyro'] = map(list, zip(*datafile.get('data/Gyro' + ext)[()]))
                        self.receiver.imu_data['imus'][i]['mag'] = map(list, zip(*datafile.get('data/Mag' + ext)[()]))
                        i += 1
                        ext = str(i + 1)

                    self.receiver.num_imus = i

                    if (self.receiver.USE_ENCODER):
                        self.receiver.imu_data['encoder'] = datafile.get('data/Encoder')[()]

            elif (filetype == "*.csv"):

                if (self.receiver.USE_ENCODER):
                    expected_non_imu_columns = 2
                else:
                    expected_non_imu_columns = 1
                self.receiver.num_imus = None
                with open(filename, 'rb') as datafile:
                    self.receiver.imu_data = {}
                    self.receiver.imu_data['timestamps'] = datafile.get('data/t')[()]
                    self.receiver.imu_data['imus'] = []
                    reader = csv.reader(datafile, delimiter=',')
                    for row in reader:
                        if (len(row) % 3 != expected_non_imu_columns):
                            self.error_slot("invalid csv file\n")
                            return
                        row_num_imus = (len(row) - 2) / 3
                        if self.receiver.num_imus is None:
                            self.receiver.num_imus = row_num_imus
                        else:
                            if (len(row) != self.receiver.num_imus):
                                self.error_slot("invalid csv file\n")
                                return
                            

            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return

            self.message_slot(filename + " loaded\n")
            self.data_saved = True
            self.buttons['save'].setEnabled(True)




############################################
#               OTHER SLOTS                #
############################################

    # CALLED WHEN REVEIVER THREAD FINISHES
    def receiver_done(self):
        self.receiver.recording = False
        self.recording = False
        self.buttons['record'].setText('Record')
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['save'].setEnabled(len(self.receiver.imu_data['timestamps']) > 0)
        self.buttons['test'].setEnabled(True)



        # CROP DATA IF PRE-TRIGGER
        if (len(self.receiver.imu_data['timestamps']) > 0):
            if (self.receiver.use_trigger):
                data_start_time = self.receiver.imu_data[-1]['time'] - (self.pre_trigger_delay * 1000);
                data_start_index = 0
                for i in range(0, len(self.receiver.imu_data)):
                    if (self.receiver.imu_data[i]['time'] > data_start_time):
                        self.receiver.imu_data = self.receiver.imu_data[i:]
                        break

        self.message_slot("done recording\n")


    def update(self):
 
        timestamps = self.receiver.imu_data['timestamps']

        if(len(timestamps) > 0):
            self.stats_time.setText('Time: %.1f' % (timestamps[-1]))

        num_samples = len(timestamps)
        self.stats_num_samples.setText('Samples: %d' % num_samples)

        if (num_samples > Am_gui.FREQ_AVERAGE_WINDOW):
            window = timestamps[-(Am_gui.FREQ_AVERAGE_WINDOW):]
            differences = [j-i for i, j in zip(window[:-1], window[1:])]
            self.true_frequency = 1000 / (sum(differences) / len(differences))
            self.stats_true_frequency.setText('Frequency: %.3f' % self.true_frequency)

        for p in self.plots:
            p.plot_slot()





    def numimus_slot(self, num_imus):
        self.num_imus = num_imus
        self.make_plots()


    # CONVENIENCE FUNCTION TO CALL MESSAGE_SLOT WITH RED TEXT
    def error_slot(self, the_string):
        self.message_slot(the_string, True)


    # CALLED BY ANYONE TO DISPLAY TEXT IN TEXT WINDOW
    def message_slot(self, the_string, red=False):
        if (red):
            self.text_window.setTextColor(QtGui.QColor(255,0,0))
        else:
            self.text_window.setTextColor(QtGui.QColor(200,200,200))
            #self.text_window.setTextColor(QtGui.QColor(0,0,0))
        self.text_window.insertPlainText(the_string)
        sb = self.text_window.verticalScrollBar();
        sb.setValue(sb.maximum());



############################################
#         OTHER FUNCTIONS (NOT SLOTS)      #
############################################

    def start_plot_slot(self):
        self.timer.start(Am_gui.PLOT_DELAY_MS)

    # START RECORDING DATA
    def record(self):
        self.recording = True
        self.receiver.recording = True
        self.data_saved = False

        self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['save'].setEnabled(False)
        #self.buttons['test'].setEnabled(False)

        self.receiver_thread.start()






    # STOP RECORDING DATA
    # THE QUIT BUTTON SLOT, STOP BUTTON SLOT, PROCESS INTERRUPT, AND TRIGGER CALL THIS FUNCTION
    # THIS FUNCTION SETS A FLAG, CAUSING THE am_rx.py PROCESS TO HALT
    # THAT WILL CAUSE THE receiver_done SLOT TO EXECUTE,
    # WHICH WILL FINISH UP STOP RECORDING DUTIES.
    # THIS FUNCTION SHOULD BE OK WITH BEING CALLED REPEATEDLY
    def stop_recording(self):
        self.receiver.recording = False
        self.timer.stop()

                                          
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)    # terminate on interrupt, will leave child process running!
    app = QApplication(sys.argv)
    ex = Am_gui()
    atexit.register(ex.stop_recording)
    ex.show()
    sys.exit(app.exec_())
          
if __name__ == '__main__':
    main()
