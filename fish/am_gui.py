#!/usr/bin/env python

import sys
import datetime
import signal
import atexit

import PyQt5.QtCore
import PyQt5.QtGui

from fish.am_rx import Am_rx
from fish.am_data import Am_data
from fish.am_plot import Am_plot
from fish.am_settings import Am_settings
from fish.am_process_dialog import Am_process_dialog

class Am_gui(PyQt5.QtGui.QWidget):

    # DISPLAYED IN WINDOW HEADER AND SUCH
    APPLICATION_NAME = 'IMU Collect 0.01'

    FREQ_AVERAGE_WINDOW = 100

    PLOT_DELAY_MS = 50

    BUTTON_WIDTH = 300

    def __init__(self, parent = None):
        super(Am_gui, self).__init__(parent)



        ########################
        #       VARIABLES      #
        ########################


        # CURRENTLY RECORDING DATA?
        self.recording = False


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


        self.plots = []

        self.timer = PyQt5.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)


        self.settings = Am_settings(self)

        self.data = Am_data(self.settings)
        # HAS THE COLLECTED DATA BEEN SAVED TO FILE?
        self.data.saved = True



        ##################################################
        #   SET UP SEPARATE THREAD FOR RECEIVING DATA    #
        ##################################################

        self.receiver_thread = PyQt5.QtCore.QThread()
        self.receiver = Am_rx(self.data, self.settings)
        self.receiver.moveToThread(self.receiver_thread)






        ##################################################
        #   CREATE GUI ELEMENTS AND ADD TO MAIN WINDOW   #
        ##################################################
        

        # CREATE BUTTONS AND ADD TO BUTTON LAYOUT

        button_layout = PyQt5.QtGui.QVBoxLayout()
        self.button_container = PyQt5.QtGui.QWidget()
        self.button_container.setLayout(button_layout)

        self.buttons['record'] = PyQt5.QtGui.QPushButton('Record')
        self.buttons['record'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])


        # self.buttons['test'] = PyQt5.QtGui.QPushButton('Test')
        # self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        # self.buttons['test'].clicked.connect(self.test_button_slot)
        # button_layout.addWidget(self.buttons['test'])

        self.buttons['process'] = PyQt5.QtGui.QPushButton('Process')
        self.buttons['process'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['process'].setToolTip('Process the current data by applying a filtering algorithm.')
        self.buttons['process'].clicked.connect(self.process_button_slot)
        button_layout.addWidget(self.buttons['process'])

        self.buttons['save'] = PyQt5.QtGui.QPushButton('Save')
        self.buttons['save'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['save'].setToolTip('Save the current data to hdf5 or csv file.')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['load'] = PyQt5.QtGui.QPushButton('Load')
        self.buttons['load'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['load'].setToolTip('Load data from an hdf5 or csv file')
        self.buttons['load'].clicked.connect(self.load_button_slot)
        button_layout.addWidget(self.buttons['load'])

        self.buttons['quit'] = PyQt5.QtGui.QPushButton('Quit')
        self.buttons['quit'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        button_layout.addWidget(self.buttons['quit'])


        # TEXT OUTPUT WINDOW

        self.text_window = PyQt5.QtGui.QTextEdit()
        self.text_window.setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.text_window.setReadOnly(True)
        #print self.text_window.minimumHeight()
        self.text_window.setMinimumHeight(150)




        # STATUS INFO

        stats_layout = PyQt5.QtGui.QVBoxLayout()
        self.stats_trigger = PyQt5.QtGui.QLabel("Trigger signal state:")
        self.stats_num_samples_buffer = PyQt5.QtGui.QLabel("Samples in buffer:")
        self.stats_num_samples_recorded = PyQt5.QtGui.QLabel("Total samples recorded:")
        self.stats_true_frequency = PyQt5.QtGui.QLabel("Sample frequency:")
        self.stats_time = PyQt5.QtGui.QLabel("Time (ms):")

        stats_layout.addWidget(self.stats_trigger)
        stats_layout.addWidget(self.stats_time)
        stats_layout.addWidget(self.stats_true_frequency)
        stats_layout.addWidget(self.stats_num_samples_recorded)
        stats_layout.addWidget(self.stats_num_samples_buffer)

        self.plots_layout = PyQt5.QtGui.QGridLayout()

        # ADD WIDGETS TO LAYOUT

        # top_layout.addLayout(self.plots_layout, 1, 1, 2, 1)
        # top_layout.addWidget(self.text_window, 3, 1, 1, 1)
        # top_layout.addWidget(self.button_container, 1, 2, 1, 1)
        # top_layout.addWidget(self.settings, 2, 2, 1, 1, QtCore.Qt.AlignTop)
        # top_layout.addLayout(stats_layout, 3, 2, 1, 1, QtCore.Qt.AlignBottom)

        panel_layout = PyQt5.QtGui.QVBoxLayout()
        panel_layout.addWidget(self.button_container)
        panel_layout.addWidget(self.hline())
        panel_layout.addWidget(self.settings)
        panel_layout.addWidget(self.hline())
        panel_layout.addLayout(stats_layout)
        panel_layout.addWidget(self.hline())
        panel_layout.addWidget(self.text_window)

        top_layout = PyQt5.QtGui.QHBoxLayout()
        top_layout.addStretch()
        top_layout.addLayout(self.plots_layout)
        top_layout.addLayout(panel_layout)




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

        self.data.message_signal.connect(self.message_slot)
        self.data.error_signal.connect(self.error_slot)


    def hline(self):
        line = PyQt5.QtGui.QFrame()
        line.setFrameShape(PyQt5.QtGui.QFrame.HLine)
        line.setFrameShadow(PyQt5.QtGui.QFrame.Sunken)
        return line


    def print_pass_fail(self, val):
        if (val):
            self.message_slot("PASS\n")
        else:
            self.error_slot("FAIL\n")

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
            plot_a = Am_plot(self.data.imu_data['imus'][i]['accel'], self.data.data_lock, self)
            plot_g = Am_plot(self.data.imu_data['imus'][i]['gyro'], self.data.data_lock, self)
            plot_m = Am_plot(self.data.imu_data['imus'][i]['mag'], self.data.data_lock, self)
            self.plots.append(plot_a)
            self.plots.append(plot_g)
            self.plots.append(plot_m)
            self.plots_layout.addWidget(plot_a, i+1, 1)
            self.plots_layout.addWidget(plot_g, i+1, 2)
            self.plots_layout.addWidget(plot_m, i+1, 3)

            label = PyQt5.QtGui.QLabel("IMU " + str(i+1))
            label.setAlignment(PyQt5.QtCore.Qt.AlignVCenter | PyQt5.QtCore.Qt.AlignCenter)
            self.plots_layout.addWidget(label, i+1, 0)


    def check_saved(self):
        if (self.data.saved):
            return True
        else:

            msgBox = PyQt5.QtGui.QMessageBox()
            msgBox.setText('Unsaved data will be lost.')
            msgBox.setIcon(QMessageBox.Warning)
            continue_btn = PyQt5.QtGui.QPushButton('Continue anyway')
            save_btn = PyQt5.QtGui.QPushButton('Save data')
            cancel_btn = PyQt5.QtGui.QPushButton('Cancel')
            msgBox.addButton(continue_btn, PyQt5.QtGui.QMessageBox.YesRole)
            msgBox.addButton(save_btn, PyQt5.QtGui.QMessageBox.YesRole)
            msgBox.addButton(cancel_btn, PyQt5.QtGui.QMessageBox.NoRole)
            msgBox.exec_()

            if (msgBox.clickedButton() == save_btn):
                return(self.save_button_slot())
            elif (msgBox.clickedButton() == continue_btn):
                return True
            else:
                return False







############################################
#              BUTTON SLOTS                #
############################################


    def process_button_slot(self):

        self.w = Am_process_dialog(self.data)
        #self.w.setGeometry(QRect(100, 100, 400, 200))
        
        self.w.finished_signal.connect(self.update)
        self.w.message_signal.connect(self.message_slot)
        self.w.error_signal.connect(self.error_slot)

        self.w.show()


    def quit_button_slot(self):
        #if (self.check_saved()):
        self.stop_recording()
        time.sleep(.2)  # let thread finish
        self.message_slot("exiting")
        self.close()


    # CAN THIS BE SIMPLIFIED BY SETTING STOP RECORDING TIME IN AM_RX INSTEAD!
    def record_button_slot(self):
        if (self.recording):
            self.stop_recording()
        else:
            #if (self.check_saved()):
            self.num_samples = 0
            self.record()

    def save_button_slot(self):

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        dlg = QFileDialog()
        filename, filetype = dlg.getSaveFileName(self, "Save data", self.last_data_path, "*.hdf5;;*.csv", options=options)

        if filename:
            filename = str(filename)
            self.last_data_path = os.path.dirname(filename)

            if (filetype == "*.hdf5"):
                if '.' not in filename:
                    filename += '.hdf5'
                self.data.save_hdf5_file(filename)

            elif (filetype == "*.csv"):
                if '.' not in filename:
                    filename += '.csv'
                self.data.save_csv_file(filename)
            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return False

            #self.message_slot("data saved to  " + filename + "\n")
            self.data.saved = True
            return True




    def load_button_slot(self):

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        filename, filetype = QFileDialog.getOpenFileName(self, "Choose a file", self.last_data_path, "*.hdf5;;*.csv", options=options)

        if filename:
            filename = str(filename)
            self.last_data_path = os.path.dirname(filename)

            if (filetype == "*.hdf5"):
                self.data.load_hdf5_file(filename)
            elif (filetype == "*.csv"):
                self.data.load_csv_file(filename)
            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return

            self.make_plots()
            for p in self.plots:
                p.plot_slot()

            #self.message_slot(filename + " loaded\n")
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
        self.buttons['process'].setEnabled(True)
        self.buttons['load'].setEnabled(True)

        self.message_slot("done recording\n")


    def update(self):
 
        timestamps = self.data.imu_data['timestamps']

        if(len(timestamps) > 0):
            self.stats_time.setText('Time (ms): %.1f' % (timestamps[-1]))

        num_samples = len(timestamps)
        self.stats_num_samples_buffer.setText('Samples in buffer: %d' % num_samples)
        self.stats_num_samples_recorded.setText('Total samples recorded: %d' % self.data.total_samples)

        state = 'ON' if self.receiver.trigger_state else 'OFF'
        self.stats_trigger.setText("Trigger signal state: " + state)


        if (num_samples > Am_gui.FREQ_AVERAGE_WINDOW):
            window = timestamps[-(Am_gui.FREQ_AVERAGE_WINDOW):]
            differences = [j-i for i, j in zip(window[:-1], window[1:])]
            self.true_frequency = 1000 / (sum(differences) / len(differences))
            self.stats_true_frequency.setText('Sample frequency: %.3f' % self.true_frequency)

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
            self.text_window.setTextColor(PyQt5.QtGui.QColor(255,0,0))
        else:
            self.text_window.setTextColor(PyQt5.QtGui.QColor(200,200,200))
            #self.text_window.setTextColor(PyQt5.QtGui.QColor(0,0,0))
        #self.text_window.insertPlainText(str(datetime.datetime.today()) + "  " + the_string)
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
        self.data.saved = False

        self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['save'].setEnabled(False)
        #self.buttons['test'].setEnabled(False)

        self.buttons['process'].setEnabled(False)
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
    ex = Am_gui()
    atexit.register(ex.stop_recording)
    ex.show()
    #ex.showMaximized()
    #sys.exit(app.exec_())
    return app.exec_()
          
