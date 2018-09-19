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

from imucapture.initialize import Initialize
from imucapture.record import Record
from imucapture.data import Data
from imucapture.settings import Settings
from imucapture.raw_data_window import Raw_data_window
from imucapture.global_data import *
from imucapture.batch_transform_dialog import Batch_transform_dialog

class Gui(PyQt5.QtWidgets.QWidget):


    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, parent = None):
        super().__init__(parent)

        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)


        # SET WINDOW TITLE
        self.setWindowTitle(Global_data.APPLICATION_FULL_NAME) 


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


        self.settings = Settings(self)


        self.raw_plot_window = None




        ##################################################
        #   CREATE GUI ELEMENTS AND ADD TO MAIN WINDOW   #
        ##################################################
        
        # CREATE BUTTONS AND ADD TO BUTTON LAYOUT

        button_layout = PyQt5.QtWidgets.QVBoxLayout()
        self.button_container = PyQt5.QtWidgets.QWidget()
        self.button_container.setLayout(button_layout)

        self.buttons['record'] = PyQt5.QtWidgets.QPushButton('Record')
        self.buttons['record'].setMaximumWidth(Gui.BUTTON_WIDTH)
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])

        # self.buttons['test'] = PyQt5.QtWidgets.QPushButton('Test')
        # self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        # self.buttons['test'].clicked.connect(self.test_button_slot)
        # button_layout.addWidget(self.buttons['test'])

        #self.buttons['save'] = PyQt5.QtWidgets.QPushButton('Save')
        #self.buttons['save'].setMaximumWidth(Gui.BUTTON_WIDTH)
        #self.buttons['save'].setToolTip('Save the current data to hdf5 file')
        #self.buttons['save'].clicked.connect(self.save_button_slot)
        #button_layout.addWidget(self.buttons['save'])
        #self.buttons['save'].setEnabled(False)

        self.buttons['load'] = PyQt5.QtWidgets.QPushButton('Load')
        self.buttons['load'].setMaximumWidth(Gui.BUTTON_WIDTH)
        self.buttons['load'].setToolTip('Load data from hdf5 file')
        self.buttons['load'].clicked.connect(self.load_button_slot)
        button_layout.addWidget(self.buttons['load'])

        self.buttons['batch_transform'] = PyQt5.QtWidgets.QPushButton('Batch transform')
        self.buttons['batch_transform'].setMaximumWidth(Gui.BUTTON_WIDTH)
        #self.buttons['batch_transform'].setToolTip('Process the current data by applying a transforming algorithm')
        self.buttons['batch_transform'].setToolTip('NOT YET IMPLEMENTED')
        #self.buttons['batch_transform'].clicked.connect(self.batch_transform_button_slot)
        self.buttons['batch_transform'].setEnabled(False)
        button_layout.addWidget(self.buttons['batch_transform'])

        self.buttons['quit'] = PyQt5.QtWidgets.QPushButton('Quit')
        self.buttons['quit'].setMaximumWidth(Gui.BUTTON_WIDTH)
        self.buttons['quit'].setToolTip('Exit the program')
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        #self.buttons['quit'].clicked.connect(self.list_widgets)
        button_layout.addWidget(self.buttons['quit'])




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

        top_layout = PyQt5.QtWidgets.QHBoxLayout()
        top_layout.addStretch()
        top_layout.addLayout(panel_layout)



        # ADD TOP LEVEL LAYOUT
        self.setLayout(top_layout)


        self.buttons['record'].setFocus()





    # FOR DEBUGGING
    def list_widgets(self):
        logging.info(len(PyQt5.QtWidgets.QApplication.topLevelWidgets()))
        logging.info(len(Global_data.data_window_list))

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

    def batch_transform_button_slot(self):
        dialog = Ic_batch_transform_dialog()
        dialog.show()



    def quit_button_slot(self):
        if (self.recording):
            self.stop_recording()
            time.sleep(.2)  # let thread finish
        logging.info("exiting")
        self.close()


    # CAN THIS BE SIMPLIFIED BY SETTING STOP RECORDING TIME IN rx INSTEAD!
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
                                                                       filter="*.hdf5",
                                                                       options=options)

        if filename:
            filename = str(filename)
            logging.info("loading " + filename)

            prefix, extension = os.path.splitext(filename)

            if (extension != '.hdf5'):
                logging.error("invalid file extension: " + extension)
                return

            data = Ic_data.from_file(filename)

            if (data is not None):

                if (data.dataset_type == 'raw'):
                    plot_window = Ic_raw_data_window(data, filename)
                if (data.dataset_type == 'transformed'):
                    plot_window = Ic_transformed_data_window(data, filename)
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

        self.settings.buffer_length_signal.disconnect(self.data.set_max_samples)

        if (self.raw_plot_window is not None):           # RECORDING WAS SUCCESSFULL
            self.data.trim_data()
            self.raw_plot_window.update()
            self.raw_plot_window.recording = False
            self.raw_plot_window.activate_buttons()

        # DONE RECORDING, DITCH THE REFERENCE TO THE CURRENT PLOT WINDOW
        self.raw_plot_window = None

        logging.info("done recording")


    # UPDATE DISPLAYED INFORMATION
    def update(self):
 
        # DISPLAY NUMBER OF SAMPLES
        self.stats_num_samples_buffer.setText('Samples in buffer: %d' % self.data.num_samples)
        self.stats_num_samples_recorded.setText('Total samples recorded: %d' % self.data.total_samples)

        # DISPLAY THE TRIGGER STATE
        self.stats_trigger.setText("Trigger signal state: " + ('ON' if self.receiver.trigger_state else 'OFF'))

        self.raw_plot_window.update()



    # START RECORDING DATA
    def record(self):
        self.recording = True

        self.asa = None
        self.numimus = None

        self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['load'].setEnabled(False)

        logging.info("initializing")

        self.initialize_thread = PyQt5.QtCore.QThread()
        self.initializer = Ic_initialize()
        self.initializer.moveToThread(self.initialize_thread)
        self.initializer.finished_signal.connect(self.initialize_thread.quit)

        self.initializer.numimus_signal.connect(self.set_num_imus)
        self.initializer.asa_signal.connect(self.set_asa)
        self.initialize_thread.started.connect(self.initializer.initialize)
        self.initialize_thread.finished.connect(self.initialize_done)
        self.initialize_thread.start()

    def set_num_imus(self, num_imus):
        self.num_imus = num_imus

    def set_asa(self, asas):
        self.mag_asas = asas

    def initialize_done(self):
        if ((self.mag_asas is None) or (self.num_imus is None)):
            logging.warning("initialization failed aborting")
            return False

        logging.info("initialization succesfull, recording data")

        self.data = Ic_data.for_recording(self.num_imus, self.settings.data_buffer_len)
        self.settings.buffer_length_signal.connect(self.data.set_max_samples)

        self.receiver_thread = PyQt5.QtCore.QThread()
        self.receiver = Ic_record(self.settings, self.data, self.mag_asas)
        self.receiver.moveToThread(self.receiver_thread)
        self.receiver.finished_signal.connect(self.receiver_thread.quit)

        self.receiver_thread.started.connect(self.receiver.record)
        self.receiver_thread.finished.connect(self.receiver_done)
        self.receiver_thread.start()

        self.raw_plot_window = Ic_raw_data_window(self.data, 'unsaved raw data')
        self.raw_plot_window.finished_signal.connect(self.stop_recording)
        self.raw_plot_window.show()
        self.raw_plot_window.recording = True
        self.timer.start(Gui.PLOT_DELAY_MS)















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
        for w in Global_data.data_window_list:
            w.close()
        event.accept()

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)    # terminate on interrupt, will leave child process running!
    app = PyQt5.QtWidgets.QApplication(sys.argv)
    ex = Gui()
    # atexit.register(ex.stop_recording)    # WE OVERRIDE closeEvent() INSTEAD
    ex.show()
    return app.exec_()
          
