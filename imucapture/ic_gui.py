#!/usr/bin/env python

import sys
import os
import datetime
import signal
import atexit
import time
import logging

import PyQt5.QtCore
import PyQt5.QtGui

from imucapture.ic_rx import Ic_rx
from imucapture.ic_data import Ic_data
from imucapture.ic_plot import Ic_plot
from imucapture.ic_settings import Ic_settings
from imucapture.ic_process_dialog import Ic_process_dialog
from imucapture.ic_global import *

class Ic_gui(PyQt5.QtGui.QWidget):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, parent = None):
        super(Ic_gui, self).__init__(parent)

        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)


        # SET WINDOW TITLE
        self.setWindowTitle(Ic_global.APPLICATION_FULL_NAME) 



        ########################
        #       VARIABLES      #
        ########################


        # CURRENTLY RECORDING DATA?
        self.recording = False


        # ALL THE BUTTONS IN THE MAIN WINDOW
        self.buttons = {}

        # NUMBER OF SAMPLES COLLECTED
        self.num_samples = 0

        # MEASURED FREQUENCY OF DATA COLLECTED
        self.true_frequency = 0.0


        self.plots = []

        self.timer = PyQt5.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)


        self.settings = Ic_settings(self)

        # THE MAIN DATA BUFFER
        self.data = Ic_data()

        # HAS THE COLLECTED DATA BEEN SAVED TO FILE?
        self.data.saved = True



        ##################################################
        #   SET UP SEPARATE THREAD FOR RECEIVING DATA    #
        ##################################################

        self.receiver_thread = PyQt5.QtCore.QThread()
        self.receiver = Ic_rx(self.data, self.settings)
        self.receiver.moveToThread(self.receiver_thread)






        ##################################################
        #   CREATE GUI ELEMENTS AND ADD TO MAIN WINDOW   #
        ##################################################
        
        # CREATE BUTTONS AND ADD TO BUTTON LAYOUT

        button_layout = PyQt5.QtGui.QVBoxLayout()
        self.button_container = PyQt5.QtGui.QWidget()
        self.button_container.setLayout(button_layout)

        self.buttons['record'] = PyQt5.QtGui.QPushButton('Record')
        self.buttons['record'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])

        # self.buttons['test'] = PyQt5.QtGui.QPushButton('Test')
        # self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        # self.buttons['test'].clicked.connect(self.test_button_slot)
        # button_layout.addWidget(self.buttons['test'])

        self.buttons['filter'] = PyQt5.QtGui.QPushButton('Filter')
        self.buttons['filter'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['filter'].setToolTip('Process the current data by applying a filtering algorithm')
        self.buttons['filter'].clicked.connect(self.filter_button_slot)
        button_layout.addWidget(self.buttons['filter'])

        self.buttons['save'] = PyQt5.QtGui.QPushButton('Save')
        self.buttons['save'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['save'].setToolTip('Save the current data to hdf5 or csv file')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['load'] = PyQt5.QtGui.QPushButton('Load')
        self.buttons['load'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['load'].setToolTip('Load data from an hdf5 or csv file')
        self.buttons['load'].clicked.connect(self.load_button_slot)
        button_layout.addWidget(self.buttons['load'])

        self.buttons['quit'] = PyQt5.QtGui.QPushButton('Quit')
        self.buttons['quit'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['quit'].setToolTip('Exit the program')
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        button_layout.addWidget(self.buttons['quit'])


        # TEXT OUTPUT WINDOW

        # self.text_window = PyQt5.QtGui.QTextEdit()
        # self.text_window.setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        # self.text_window.setReadOnly(True)
        # self.text_window.setMinimumHeight(150)


        # STATUS INFO

        stats_layout = PyQt5.QtGui.QVBoxLayout()
        self.stats_trigger = PyQt5.QtGui.QLabel("Trigger signal state:")
        self.stats_time = PyQt5.QtGui.QLabel("Time (ms):")
        self.stats_true_frequency = PyQt5.QtGui.QLabel("Sample frequency:")
        self.stats_num_samples_recorded = PyQt5.QtGui.QLabel("Total samples recorded:")
        self.stats_num_samples_buffer = PyQt5.QtGui.QLabel("Samples in buffer:")

        stats_layout.addWidget(self.stats_trigger)
        stats_layout.addWidget(self.stats_time)
        stats_layout.addWidget(self.stats_true_frequency)
        stats_layout.addWidget(self.stats_num_samples_recorded)
        stats_layout.addWidget(self.stats_num_samples_buffer)

        self.plots_layout = PyQt5.QtGui.QGridLayout()

        # ADD WIDGETS TO LAYOUT

        panel_layout = PyQt5.QtGui.QVBoxLayout()
        panel_layout.addWidget(self.button_container)
        panel_layout.addWidget(self.hline())
        panel_layout.addWidget(self.settings)
        panel_layout.addWidget(self.hline())
        panel_layout.addLayout(stats_layout)
        # panel_layout.addWidget(self.hline())
        # panel_layout.addWidget(self.text_window)

        top_layout = PyQt5.QtGui.QHBoxLayout()
        top_layout.addStretch()
        top_layout.addLayout(self.plots_layout)
        top_layout.addLayout(panel_layout)



        # ADD TOP LEVEL LAYOUT
        self.setLayout(top_layout)


        self.buttons['record'].setFocus()






        ########################
        #    QT CONNECTIONS    #
        ########################


        self.receiver.finished_signal.connect(self.receiver_thread.quit)

        self.receiver.recording_signal.connect(self.start_plot_slot)

        # COLLECT DATA FROM Ic_rx() i.e. from arduino
        self.receiver_thread.started.connect(self.receiver.run)

        self.receiver_thread.finished.connect(self.receiver_done)

        self.receiver.numimus_signal.connect(self.numimus_slot)



    def hline(self):
        line = PyQt5.QtGui.QFrame()
        line.setFrameShape(PyQt5.QtGui.QFrame.HLine)
        line.setFrameShadow(PyQt5.QtGui.QFrame.Sunken)
        return line



    # thanks to ekhumoro for this function
    # http://stackoverflow.com/questions/9374063/remove-widgets-and-layout-as-well
    def clear_layout(self, layout):
        if layout is not None:
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
                else:
                    self.clearLayout(item.layout())


    def make_plots(self):

        self.clear_layout(self.plots_layout)

        label = PyQt5.QtGui.QLabel("Accel.")
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 1)

        label = PyQt5.QtGui.QLabel("Gyro.")
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 2)

        label = PyQt5.QtGui.QLabel("Mag.")
        label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 3)

        self.plots = []
        for i in (range(0, self.data.num_imus)):
            plot_a = Ic_plot(self.data.imu_data['imus'][i]['accel'], self.data.data_lock, self)
            plot_g = Ic_plot(self.data.imu_data['imus'][i]['gyro'], self.data.data_lock, self)
            plot_m = Ic_plot(self.data.imu_data['imus'][i]['mag'], self.data.data_lock, self)
            self.plots.append(plot_a)
            self.plots.append(plot_g)
            self.plots.append(plot_m)
            self.plots_layout.addWidget(plot_a, i+1, 1)
            self.plots_layout.addWidget(plot_g, i+1, 2)
            self.plots_layout.addWidget(plot_m, i+1, 3)

            label = PyQt5.QtGui.QLabel("IMU " + str(i+1))
            label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
            self.plots_layout.addWidget(label, i+1, 0)








############################################
#              BUTTON SLOTS                #
############################################


    def filter_button_slot(self):

        self.w = Ic_process_dialog(self.data)
        #self.w.setGeometry(QRect(100, 100, 400, 200))
        
        self.w.finished_signal.connect(self.update)

        self.w.show()


    def quit_button_slot(self):
        self.stop_recording()
        time.sleep(.2)  # let thread finish
        logging.info("exiting")
        self.close()


    # CAN THIS BE SIMPLIFIED BY SETTING STOP RECORDING TIME IN AM_RX INSTEAD!
    def record_button_slot(self):
        if (self.recording):
            self.stop_recording()
        else:
            self.num_samples = 0
            self.record()



    # GET A FILENAME AND TYPE FROM THE USER AND THEN SAVE THE DATA BUFFER
    def save_button_slot(self):

        options = PyQt5.QtGui.QFileDialog.Options() | PyQt5.QtGui.QFileDialog.DontUseNativeDialog
        filename, filetype = PyQt5.QtGui.QFileDialog.getSaveFileName(parent=self,
                                                                     caption="Save data",
                                                                     directory=Ic_global.last_file_path,
                                                                     filter="*.csv;;*.hdf5",
                                                                     options=options)

        if filename:
            filename = str(filename)
            Ic_global.last_data_path = os.path.dirname(filename)

            if (filetype == "*.hdf5"):
                if '.' not in filename:
                    filename += '.hdf5'
                self.data.save_hdf5_file(filename)

            elif (filetype == "*.csv"):
                if '.' not in filename:
                    filename += '.csv'
                self.data.save_csv_file(filename)
            else:
                logging.error("invalid file type: " + filetype)
                return False

            logging.info("saved " + filename)
            self.data.saved = True
            return True




    # GET A FILENAME FROM THE USER AND THEN LOAD THE FILE TO THE DATA BUFFER
    def load_button_slot(self):

        options = PyQt5.QtGui.QFileDialog.Options() | PyQt5.QtGui.QFileDialog.DontUseNativeDialog
        filename, searchtype = PyQt5.QtGui.QFileDialog.getOpenFileName(parent=self,
                                                                       caption="Choose a file",
                                                                       directory=Ic_global.last_file_path,
                                                                       filter="*.csv *.hdf5",
                                                                       options=options)

        if filename:
            filename = str(filename)
            logging.info("loading " + filename)

            prefix, extension = os.path.splitext(filename)
            Ic_global.last_data_path = os.path.dirname(filename)


            if (extension == '.hdf5'):
                if not self.data.load_hdf5_file(filename):
                    logging.error("invalid hdf5 file")
                    return
            elif (extension == '.csv'):
                if not self.data.load_csv_file(filename):
                    logging.error("invalid csv file")
                    return
            else:
                logging.error("invalid file extension: " + extension)
                return

            self.make_plots()
            for p in self.plots:
                p.plot_slot()

            logging.info(filename + " loaded")
            self.data.saved = True
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
        self.buttons['save'].setEnabled(self.data.has_data())
        #self.buttons['test'].setEnabled(True)
        self.buttons['filter'].setEnabled(True)
        self.buttons['load'].setEnabled(True)

        logging.info("done recording")


    # UPDATE DISPLAYED INFORMATION
    def update(self):
 
        # DISPLAY TIME
        timestamps = self.data.imu_data['timestamps']
        if(len(timestamps) > 0):
            self.stats_time.setText('Time (ms): %.1f' % (timestamps[-1]))

        # DISPLAY NUMBER OF SAMPLES
        num_samples = len(timestamps)
        self.stats_num_samples_buffer.setText('Samples in buffer: %d' % num_samples)
        self.stats_num_samples_recorded.setText('Total samples recorded: %d' % self.data.total_samples)

        # DISPLAY THE TRIGGER STATE
        self.stats_trigger.setText("Trigger signal state: " + ('ON' if self.receiver.trigger_state else 'OFF'))

        # CALCULATE AND DISPLAY THE ACTUAL CURRENT MEASUREMENT FREQUENCY
        if (num_samples > Ic_gui.FREQ_AVERAGE_WINDOW):
            window = timestamps[-(Ic_gui.FREQ_AVERAGE_WINDOW):]
            differences = [j-i for i, j in zip(window[:-1], window[1:])]
            self.true_frequency = 1000 / (sum(differences) / len(differences))
            self.stats_true_frequency.setText('Sample frequency: %.3f' % self.true_frequency)

        # REFRESH ALL PLOTS
        for p in self.plots:
            p.plot_slot()


    # SYNC NUMBER OF IMUS WITH RECEIVER AND CREATE THE CORRECT NUMBER OF PLOTS
    def numimus_slot(self, num_imus):
        self.num_imus = num_imus
        self.make_plots()



############################################
#         OTHER FUNCTIONS (NOT SLOTS)      #
############################################

    def start_plot_slot(self):
        self.timer.start(Ic_gui.PLOT_DELAY_MS)

    # START RECORDING DATA
    def record(self):
        self.recording = True
        self.receiver.recording = True
        self.data.saved = False

        self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['save'].setEnabled(False)
        #self.buttons['test'].setEnabled(False)

        self.buttons['filter'].setEnabled(False)
        self.buttons['load'].setEnabled(False)

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
    app = PyQt5.QtGui.QApplication(sys.argv)
    ex = Ic_gui()
    atexit.register(ex.stop_recording)
    ex.show()
    #ex.showMaximized()
    #sys.exit(app.exec_())
    return app.exec_()
          