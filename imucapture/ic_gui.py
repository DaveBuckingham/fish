#!/usr/bin/env python

import sys
import os
import datetime
import signal
import atexit
import time
import logging

import PyQt5.QtCore
import PyQt5.QtWidgets

from imucapture.ic_rx import Ic_rx
from imucapture.ic_data import Ic_data
from imucapture.ic_plot import Ic_plot
from imucapture.ic_settings import Ic_settings
from imucapture.ic_raw_data_window import Ic_raw_data_window
from imucapture.ic_global import *
from imucapture.ic_batch_transform_dialog import Ic_batch_transform_dialog

class Ic_gui(PyQt5.QtWidgets.QWidget):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, parent = None):
        super().__init__(parent)

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

        self.data = None


        self.raw_plot_window = None




        ##################################################
        #   CREATE GUI ELEMENTS AND ADD TO MAIN WINDOW   #
        ##################################################
        
        # CREATE BUTTONS AND ADD TO BUTTON LAYOUT

        button_layout = PyQt5.QtWidgets.QVBoxLayout()
        self.button_container = PyQt5.QtWidgets.QWidget()
        self.button_container.setLayout(button_layout)

        self.buttons['record'] = PyQt5.QtWidgets.QPushButton('Record')
        self.buttons['record'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])

        # self.buttons['test'] = PyQt5.QtWidgets.QPushButton('Test')
        # self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        # self.buttons['test'].clicked.connect(self.test_button_slot)
        # button_layout.addWidget(self.buttons['test'])

        #self.buttons['save'] = PyQt5.QtWidgets.QPushButton('Save')
        #self.buttons['save'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        #self.buttons['save'].setToolTip('Save the current data to hdf5 or csv file')
        #self.buttons['save'].clicked.connect(self.save_button_slot)
        #button_layout.addWidget(self.buttons['save'])
        #self.buttons['save'].setEnabled(False)

        self.buttons['load'] = PyQt5.QtWidgets.QPushButton('Load')
        self.buttons['load'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['load'].setToolTip('Load data from an hdf5 or csv file')
        self.buttons['load'].clicked.connect(self.load_button_slot)
        button_layout.addWidget(self.buttons['load'])

        self.buttons['transform'] = PyQt5.QtWidgets.QPushButton('Batch transform')
        self.buttons['transform'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['transform'].setToolTip('Process the current data by applying a transforming algorithm')
        self.buttons['transform'].clicked.connect(self.transform_button_slot)
        button_layout.addWidget(self.buttons['transform'])

        self.buttons['quit'] = PyQt5.QtWidgets.QPushButton('Quit')
        self.buttons['quit'].setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        self.buttons['quit'].setToolTip('Exit the program')
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        #self.buttons['quit'].clicked.connect(self.list_widgets)
        button_layout.addWidget(self.buttons['quit'])


        # TEXT OUTPUT WINDOW

        # self.text_window = PyQt5.QtWidgets.QTextEdit()
        # self.text_window.setMaximumWidth(Ic_gui.BUTTON_WIDTH)
        # self.text_window.setReadOnly(True)
        # self.text_window.setMinimumHeight(150)


        # STATUS INFO

        stats_layout = PyQt5.QtWidgets.QVBoxLayout()
        self.stats_trigger = PyQt5.QtWidgets.QLabel("Trigger signal state:")
        self.stats_time = PyQt5.QtWidgets.QLabel("Time (ms):")
        self.stats_true_frequency = PyQt5.QtWidgets.QLabel("Sample frequency:")
        self.stats_num_samples_recorded = PyQt5.QtWidgets.QLabel("Total samples recorded:")
        self.stats_num_samples_buffer = PyQt5.QtWidgets.QLabel("Samples in buffer:")

        stats_layout.addWidget(self.stats_trigger)
        stats_layout.addWidget(self.stats_time)
        stats_layout.addWidget(self.stats_true_frequency)
        stats_layout.addWidget(self.stats_num_samples_recorded)
        stats_layout.addWidget(self.stats_num_samples_buffer)


        # ADD WIDGETS TO LAYOUT

        panel_layout = PyQt5.QtWidgets.QVBoxLayout()
        panel_layout.addWidget(self.button_container)
        panel_layout.addWidget(self.hline())
        panel_layout.addWidget(self.settings)
        panel_layout.addWidget(self.hline())
        panel_layout.addLayout(stats_layout)
        # panel_layout.addWidget(self.hline())
        # panel_layout.addWidget(self.text_window)

        top_layout = PyQt5.QtWidgets.QHBoxLayout()
        top_layout.addStretch()
        top_layout.addLayout(panel_layout)



        # ADD TOP LEVEL LAYOUT
        self.setLayout(top_layout)


        self.buttons['record'].setFocus()





    # FOR DEBUGGING
    def list_widgets(self):
        logging.info(len(PyQt5.QtWidgets.QApplication.topLevelWidgets()))
        logging.info(len(Ic_global.data_window_list))
        #for w in PyQt5.QtWidgets.QApplication.topLevelWidgets():
        #    w.show()
            




    def hline(self):
        line = PyQt5.QtWidgets.QFrame()
        line.setFrameShape(PyQt5.QtWidgets.QFrame.HLine)
        line.setFrameShadow(PyQt5.QtWidgets.QFrame.Sunken)
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





############################################
#              BUTTON SLOTS                #
############################################

    def transform_button_slot(self):
        dialog = Ic_batch_transform_dialog()
        dialog.show()



    def quit_button_slot(self):
        if (self.recording):
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





    # GET A FILENAME FROM THE USER AND THEN LOAD THE FILE TO THE DATA BUFFER
    def load_button_slot(self):

        options = PyQt5.QtWidgets.QFileDialog.Options() | PyQt5.QtWidgets.QFileDialog.DontUseNativeDialog
        filename, searchtype = PyQt5.QtWidgets.QFileDialog.getOpenFileName(parent=self,
                                                                       caption="Choose a file",
                                                                       directory=Ic_global.last_file_path,
                                                                       filter="*.csv *.hdf5",
                                                                       options=options)

        if filename:
            filename = str(filename)
            logging.info("loading " + filename)

            prefix, extension = os.path.splitext(filename)
            Ic_global.last_data_path = os.path.dirname(filename)

            data = Ic_data()

            if (extension == '.hdf5'):
                if not data.load_hdf5_file(filename):
                    logging.error("invalid hdf5 file")
                    return
            elif (extension == '.csv'):
                if not data.load_csv_file(filename):
                    logging.error("invalid csv file")
                    return
            else:
                logging.error("invalid file extension: " + extension)
                return

            plot_window = Ic_raw_data_window(data, filename)
            plot_window.update()
            plot_window.activate_buttons()
            plot_window.show()


            logging.info(filename + " loaded")




############################################
#               OTHER SLOTS                #
############################################

    # CALLED WHEN REVEIVER THREAD FINISHES
    def receiver_done(self):
        self.recording = False
        self.buttons['record'].setText('Record')
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['load'].setEnabled(True)

        if (self.raw_plot_window is not None):           # RECORDING WAS SUCCESSFULL
            self.raw_plot_window.activate_buttons()

        # DONE RECORDING, DITCH THE REFERENCE TO THE CURRENT PLOT WINDOW
        self.raw_plot_window = None

        logging.info("done recording")


    # UPDATE DISPLAYED INFORMATION
    def update(self):
 
        # DISPLAY TIME
        #timestamps = self.data.imu_data['timestamps']
        #if(len(timestamps) > 0):
        #    self.stats_time.setText('Time (ms): %.1f' % (timestamps[-1]))

        # DISPLAY NUMBER OF SAMPLES
        #num_samples = len(timestamps)
        self.stats_num_samples_buffer.setText('Samples in buffer: %d' % self.data.num_samples)
        self.stats_num_samples_recorded.setText('Total samples recorded: %d' % self.data.total_samples)

        # DISPLAY THE TRIGGER STATE
        self.stats_trigger.setText("Trigger signal state: " + ('ON' if self.receiver.trigger_state else 'OFF'))

        # CALCULATE AND DISPLAY THE ACTUAL CURRENT MEASUREMENT FREQUENCY
        #if (self.data.num_samples > Ic_gui.FREQ_AVERAGE_WINDOW):
        
            # DOESN'T WORK WITH DEQUE
            #window = timestamps[-(Ic_gui.FREQ_AVERAGE_WINDOW):]

            #window = [timestamps[i] for i in range(len(timestamps)-(Ic_gui.FREQ_AVERAGE_WINDOW), len(timestamps))]

            #differences = [j-i for i, j in zip(window[:-1], window[1:])]
            #self.true_frequency = 1000 / (sum(differences) / len(differences))
            #self.stats_true_frequency.setText('Sample frequency: %.3f' % self.true_frequency)

        self.raw_plot_window.update()


    # GET RID OF THIS ??

    # SYNC NUMBER OF IMUS WITH RECEIVER AND CREATE THE CORRECT NUMBER OF PLOTS
    def numimus_slot(self, num_imus):
        self.num_imus = num_imus



############################################
#         OTHER FUNCTIONS (NOT SLOTS)      #
############################################

    def start_plot_slot(self):
        self.timer.start(Ic_gui.PLOT_DELAY_MS)
        self.raw_plot_window = Ic_raw_data_window(self.data, 'unsaved raw data')
        self.raw_plot_window.finished_signal.connect(self.stop_recording)
        self.raw_plot_window.show()

    # START RECORDING DATA
    def record(self):
        self.recording = True

        self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['load'].setEnabled(False)

        self.data = Ic_data()

        self.receiver_thread = PyQt5.QtCore.QThread()
        self.receiver = Ic_rx(self.data, self.settings)
        self.receiver.moveToThread(self.receiver_thread)
        self.receiver.finished_signal.connect(self.receiver_thread.quit)
        self.receiver.recording_signal.connect(self.start_plot_slot)
        self.receiver_thread.started.connect(self.receiver.run)
        self.receiver_thread.finished.connect(self.receiver_done)
        self.receiver.numimus_signal.connect(self.numimus_slot)

        self.receiver_thread.start()




    # STOP RECORDING DATA
    # THE QUIT BUTTON SLOT, STOP BUTTON SLOT, PROCESS INTERRUPT, AND TRIGGER CALL THIS FUNCTION
    # ALSO CALLED IF PLOT WINDOW IS CLOSED WHILE RECORDING
    # THIS FUNCTION SETS A FLAG, CAUSING THE am_rx.py PROCESS TO HALT
    # THAT WILL CAUSE THE receiver_done SLOT TO EXECUTE,
    # WHICH WILL FINISH UP STOP RECORDING DUTIES.
    # THIS FUNCTION SHOULD BE OK WITH BEING CALLED REPEATEDLY
    def stop_recording(self):
        self.receiver.recording = False
        self.timer.stop()


    # CALLED WHEN MAIN WINDOW CLOSES
    def closeEvent(self, event):
        # STOP ANY RECORDING IN PROGRESS
        if(self.recording):
            self.stop_recording()
        # CLOSE ALL DATA WINDOWS
        for w in Ic_global.data_window_list:
            w.close()
        event.accept()

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)    # terminate on interrupt, will leave child process running!
    app = PyQt5.QtWidgets.QApplication(sys.argv)
    ex = Ic_gui()
    # atexit.register(ex.stop_recording)    # WE OVERRIDE closeEvent() INSTEAD
    ex.show()
    return app.exec_()
          
