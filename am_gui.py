#!/usr/bin/env python

# USE PYTHON 2.6 OR LATER

import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from am_rx import *
from am_plot import *
from am_settings import *
from collections import namedtuple
import time
import h5py
import signal
from collections import deque

class Am_ui(QWidget):

    # DISPLAYED IN WINDOW HEADER AND SUCH
    APPLICATION_NAME = 'IMU Collect 0.01'

    FREQ_AVERAGE_WINDOW = 100

    # SIGNAL TO RESETS ALL THE PLOTS
    clear_plots_signal = pyqtSignal()


    def __init__(self, parent = None):
        super(Am_ui, self).__init__(parent)



        ########################
        #       VARIABLES      #
        ########################

        # HAS THE COLLECTED DATA BEEN SAVED TO FILE?
        self.data_saved = True

        # CURRENTLY RECORDING DATA?
        self.recording = False

        # IF FALSE, RECORD BUTTON TO START AND STOP RECORDING
        # IF TRUE, SECOND CLICK CAPTURES DATA IN RANGE FROM PRE TO POST DELAY.
        self.use_trigger = False
        self.pre_trigger_delay = 3
        self.post_trigger_delay = 3

        # ALL THE BUTTONS IN THE MAIN WINDOW
        self.buttons = {}


        # TIMESTAMPS OF COLLECTED DATA.
        self.timestamps   = []

        # NUMBER OF SAMPLES COLLECTED
        self.num_samples = 0

        # MEASURED FREQUENCY OF DATA COLLECTED
        self.true_frequency = 0.0

        # HOLD ALL VISUAL ELEMENTS IN GUI MAIN WINDOW
        top_layout = QGridLayout()




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
        # self.buttons['record'].setStyleSheet("background-color: red")


        self.buttons['test'] = QPushButton('Test')
        self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        self.buttons['test'].clicked.connect(self.test_button_slot)
        button_layout.addWidget(self.buttons['test'])

        self.buttons['save'] = QPushButton('Save')
        self.buttons['save'].setToolTip('Save recorded data')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['quit'] = QPushButton('Quit')
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        button_layout.addWidget(self.buttons['quit'])


        # TEXT OUTPUT WINDOW

        self.text_window = QTextEdit()
        self.text_window.setReadOnly(True)
        #print self.text_window.minimumHeight()
        #self.text_window.setMinimumHeight(50)


        # GRAPHS

        self.plot_a0 = Am_plot()
        self.plot_a1 = Am_plot()
        self.plot_g0 = Am_plot()
        self.plot_g1 = Am_plot()
        self.plot_m0 = Am_plot()
        self.plot_m1 = Am_plot()




        # SETTINGS

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


        # ADD WIDGETS TO LAYOUT

        top_layout.addWidget(self.plot_a0, 1, 1)
        top_layout.addWidget(self.plot_a1, 1, 2)
        top_layout.addWidget(self.plot_g0, 2, 1)
        top_layout.addWidget(self.plot_g1, 2, 2)
        top_layout.addWidget(self.plot_m0, 3, 1)
        top_layout.addWidget(self.plot_m1, 3, 2)

        top_layout.addWidget(self.text_window, 4, 1, 1, 2)
        top_layout.addLayout(stats_layout, 5, 1, 1, 2)

        top_layout.addWidget(self.button_container, 1, 3, 3, 1)
        top_layout.addWidget(self.settings, 4, 3)


        # GRAPH LABELS

        label = QtGui.QLabel("IMU 1")
        label.setAlignment(QtCore.Qt.AlignBottom | QtCore.Qt.AlignCenter)
        top_layout.addWidget(label, 0, 1)

        label = QtGui.QLabel("IMU 2")
        label.setAlignment(QtCore.Qt.AlignBottom | QtCore.Qt.AlignCenter)
        top_layout.addWidget(label, 0, 2)

        label = QtGui.QLabel("Accel.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        top_layout.addWidget(label, 1, 0)

        label = QtGui.QLabel("Gyro.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        top_layout.addWidget(label, 2, 0)

        label = QtGui.QLabel("Mag.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        top_layout.addWidget(label, 3, 0)


        # ADD TOP LEVEL LAYOUT
        self.setLayout(top_layout)


        # SET WINDOW TITLE
        self.setWindowTitle(Am_ui.APPLICATION_NAME) 



        ##################################################
        #   SET UP SEPARATE THREAD FOR RECEIVING DATA    #
        ##################################################

        self.receiver_thread = QThread()
        self.receiver = Am_rx()
        self.receiver.moveToThread(self.receiver_thread)



        ########################
        #    QT CONNECTIONS    #
        ########################

        self.clear_plots_signal.connect(self.plot_a0.clear_slot)
        self.clear_plots_signal.connect(self.plot_a1.clear_slot)
        self.clear_plots_signal.connect(self.plot_g0.clear_slot)
        self.clear_plots_signal.connect(self.plot_g1.clear_slot)
        self.clear_plots_signal.connect(self.plot_m0.clear_slot)
        self.clear_plots_signal.connect(self.plot_m1.clear_slot)

        self.receiver.finished_signal.connect(self.receiver_thread.quit)

        # USE TO TEST WITHOUT ARDUINO, RANDOMLY GENERATED DATA
        # self.receiver_thread.started.connect(self.receiver.run_fake)

        # COLLECT DATA FROM Am_rx() i.e. from arduino
        self.receiver_thread.started.connect(self.receiver.run)

        self.receiver_thread.finished.connect(self.receiver_done)

        self.receiver.timestamp_signal.connect(self.timestamp_slot)
        self.receiver.plot_a0_signal.connect(self.plot_a0.data_slot)
        self.receiver.plot_a1_signal.connect(self.plot_a1.data_slot)
        self.receiver.plot_g0_signal.connect(self.plot_g0.data_slot)
        self.receiver.plot_g1_signal.connect(self.plot_g1.data_slot)
        self.receiver.plot_m0_signal.connect(self.plot_m0.data_slot)
        self.receiver.plot_m1_signal.connect(self.plot_m1.data_slot)

        self.receiver.message_signal.connect(self.message_slot)
        self.receiver.error_signal.connect(self.error_slot)



    def print_pass_fail(self, val):
        if (val):
            self.message_slot("PASS\n")
        else:
            self.error_slot("FAIL\n")



############################################
#              BUTTON SLOTS                #
############################################


    def test_button_slot(self):

        results = self.receiver.test()

        if (not results):
            self.error_slot("Arduino com error.\n")

        else:

            self.message_slot("imu1 communication test...")
            self.print_pass_fail(results[0])

            self.message_slot("imu2 communication test...")
            self.print_pass_fail(results[1])

            self.message_slot("imu1 self test...")
            self.message_slot("not implemented\n")

            self.message_slot("imu2 self test...")
            self.message_slot("not implemented\n")

            self.message_slot("mag1 self test...")
            self.message_slot("not implemented\n")

            self.message_slot("mag2 self test...")
            self.message_slot("not implemented\n")



    def quit_button_slot(self):
        result = (QMessageBox.question(self,
                                       Am_ui.APPLICATION_NAME,
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
        filename = QFileDialog.getSaveFileName(self, 'Save recorded data', '', '*.hdf5')
        if filename:
            if ( (len(filename) < 5) or (filename[-5:].toLower() != '.hdf5') ):
                filename += '.hdf5'

            datafile = h5py.File(str(filename), 'w')
            save_data = datafile.create_group("data")
            save_data.create_dataset('t',      data=[x['time']   for x in self.receiver.data])
            save_data.create_dataset('Accel',  data=[x['accel0'] for x in self.receiver.data])
            save_data.create_dataset('Accel2', data=[x['accel1'] for x in self.receiver.data])
            save_data.create_dataset('Gyro',   data=[x['gyro0']  for x in self.receiver.data])
            save_data.create_dataset('Gyro2',  data=[x['gyro1']  for x in self.receiver.data])
            save_data.create_dataset('Mag',    data=[x['mag0']   for x in self.receiver.data])
            save_data.create_dataset('Mag2',   data=[x['mag1']   for x in self.receiver.data])
            datafile.close()

            self.message_slot("data saved to  " + filename + "\n")
            self.data_saved = True
            #self.buttons['save'].setEnabled(False)




############################################
#               OTHER SLOTS                #
############################################

    # CALLED WHEN REVEIVER THREAD FINISHES
    def receiver_done(self):
        self.receiver.recording = False
        self.recording = False
        self.buttons['record'].setText('Record')
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['save'].setEnabled(len(self.timestamps) > 0)
        # self.settings.setEnabled(True)
        self.buttons['test'].setEnabled(True)


        # CROP DATA IF PRE-TRIGGER
        if (len(self.timestamps) > 0):
            if (self.use_trigger):
                data_start_time = self.receiver.data[-1]['time'] - (self.pre_trigger_delay * 1000);
                data_start_index = 0
                for i in range(0, len(self.receiver.data)):
                    if (self.receiver.data[i]['time'] > data_start_time):
                        self.receiver.data = self.receiver.data[i:]
                        break

        self.message_slot("done recording\n")




    # CALLED BY am_rx.py FOR EVERY SAMPLE
    def timestamp_slot(self, timestamp):

        self.timestamps.append(timestamp)

        self.stats_time.setText('Time: %.1f' % (timestamp))

        self.num_samples += 1
        self.stats_num_samples.setText('Samples: %d' % self.num_samples)

        if (self.num_samples > Am_ui.FREQ_AVERAGE_WINDOW):
            # self.true_frequency = (Am_ui.FREQ_AVERAGE_WINDOW / (self.timestamps[-1] - self.timestamps[-(Am_ui.FREQ_AVERAGE_WINDOW + 1)])) * 1000
            window = self.timestamps[-(Am_ui.FREQ_AVERAGE_WINDOW):]
            differences = [j-i for i, j in zip(window[:-1], window[1:])]
            self.true_frequency = 1000 / (sum(differences) / len(differences))
            self.stats_true_frequency.setText('Frequency: %.3f' % self.true_frequency)



    # CONVENIENCE FUNCTION TO CALL MESSAGE_SLOT WITH RED TEXT
    def error_slot(self, the_string):
        self.message_slot(the_string, True)


    # CALLED BY ANYONE TO DISPLAY TEXT IN TEXT WINDOW
    def message_slot(self, the_string, red=False):
        # self.text_window.setTextColor(QtGui.QColor(120, 120, 120))
        # self.text_window.insertPlainText("\n" + time.strftime("%c") + "    ")

        if (red):
            self.text_window.setTextColor(QtGui.QColor(255,0,0))
        else:
            self.text_window.setTextColor(QtGui.QColor(0,0,0))
        self.text_window.insertPlainText(the_string)
        sb = self.text_window.verticalScrollBar();
        sb.setValue(sb.maximum());



############################################
#         OTHER FUNCTIONS (NOT SLOTS)      #
############################################


    # START RECORDING DATA
    def record(self):
        self.recording = True
        self.receiver.recording = True
        self.data_saved = False


        self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['save'].setEnabled(False)
        # self.settings.setEnabled(False)
        self.buttons['test'].setEnabled(False)

        self.clear_plots_signal.emit()

        self.receiver_thread.start()


    # STOP RECORDING DATA
    # THE QUIT BUTTON SLOT CALLS THIS FUNCTION
    # THIS FUNCTION SETS A FLAG, CAUSING THE am_rx.py PROCESS TO HALT
    # THAT WILL CAUSE THE receiver_done SLOT TO EXECUTE,
    # WHICH WILL FINISH UP STOP RECORDING DUTIES.
    def stop_recording(self):
        self.receiver.recording = False

                                          
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)    # terminate on interrupt
    app = QApplication(sys.argv)
    ex = Am_ui()
    ex.show()
    sys.exit(app.exec_())
          
if __name__ == '__main__':
    main()
