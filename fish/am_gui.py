#!/usr/bin/env python

# USE PYTHON 2.6 OR LATER

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from fish.am_rx import *
from fish.am_plot import *
from fish.am_settings import *
#from fish.am_process import *
from fish.am_process_dialog import *
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

    BUTTON_WIDTH = 300

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

        self.plots = []

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
        self.buttons['record'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])


        # self.buttons['test'] = QPushButton('Test')
        # self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        # self.buttons['test'].clicked.connect(self.test_button_slot)
        # button_layout.addWidget(self.buttons['test'])

        self.buttons['process'] = QPushButton('Process')
        self.buttons['process'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['process'].setToolTip('Process data')
        self.buttons['process'].clicked.connect(self.process_button_slot)
        button_layout.addWidget(self.buttons['process'])

        self.buttons['save'] = QPushButton('Save')
        self.buttons['save'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['save'].setToolTip('Save recorded data')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['load'] = QPushButton('Load')
        self.buttons['load'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
        self.buttons['load'].setToolTip('Load recorded data')
        self.buttons['load'].clicked.connect(self.load_button_slot)
        button_layout.addWidget(self.buttons['load'])

        self.buttons['quit'] = QPushButton('Quit')
        self.buttons['quit'].setMaximumWidth(Am_gui.BUTTON_WIDTH)
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

        label = QtGui.QLabel("Accel.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 1)

        label = QtGui.QLabel("Gyro.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 2)

        label = QtGui.QLabel("Mag.")
        label.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignCenter)
        self.plots_layout.addWidget(label, 0, 3)

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


    def check_saved(self):
        if (self.data_saved):
            return True
        else:

            msgBox = QtGui.QMessageBox()
            msgBox.setText('Unsaved data will be lost.')
            msgBox.setIcon(QMessageBox.Warning)
            continue_btn = QtGui.QPushButton('Continue anyway')
            save_btn = QtGui.QPushButton('Save data')
            cancel_btn = QtGui.QPushButton('Cancel')
            msgBox.addButton(continue_btn, QtGui.QMessageBox.YesRole)
            msgBox.addButton(save_btn, QtGui.QMessageBox.YesRole)
            msgBox.addButton(cancel_btn, QtGui.QMessageBox.NoRole)
            msgBox.exec_()

            if (msgBox.clickedButton() == save_btn):
                return(self.save_button_slot())
            elif (msgBox.clickedButton() == continue_btn):
                return True
            else:
                return False



    def process_data(self):
        if (self.receiver.has_data()):
            for i in range(0, len(self.receiver.imu_data['timestamps'])):
                for j in range(0, len(self.receiver.imu_data['imus'])):
                    for k in range(0, 3):
                        self.receiver.imu_data['imus'][j]['accel'][k][i] *= -2
                        self.receiver.imu_data['imus'][j]['gyro'][k][i] *= -1
                        self.receiver.imu_data['imus'][j]['mag'][k][i] *= 2




############################################
#              BUTTON SLOTS                #
############################################

    def batch_process(self):

        # LET THE USER SET THIS
        suffix = "_processed"

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        (filename_list, types) = QFileDialog.getOpenFileNames(self, "Select files to process", self.last_data_path, "*.hdf5 *.csv", options=options)

        if(filename_list):
            for filename in filename_list:
                (base, extension) = os.path.splitext(filename)
                if ((extension == ".hdf5") or (extension == ".csv")):
                    if (extension == ".hdf5"):
                        self.load_hdf5_file(filename)
                        self.process_data()
                        out_name = base + suffix + extension
                        self.save_hdf5_file(out_name)
                    elif (extension == ".csv"):
                        self.load_csv_file(filename)
                        self.process_data()
                        out_name = base + suffix + extension
                        self.save_csv_file(out_name)
            self.receiver.reset_data(0)
            self.data_saved = True
            self.make_plots()






    def process_button_slot(self):

        self.w = Am_process_dialog(True)
        # self.w = Am_process_dialog(self.receiver.has_data())
        #self.w.setGeometry(QRect(100, 100, 400, 200))
        self.w.show()

        #if (not self.receiver.has_data()):
        #    self.batch_process()
        #else:
        #    msgBox = QtGui.QMessageBox()
        #    msgBox.setText('Do you want to process previously saved data or the currently loaded data?')
        #    msgBox.setIcon(QMessageBox.Question)
        #    use_saved_btn = QtGui.QPushButton('Select files for batch processing')
        #    use_current_btn = QtGui.QPushButton('Use current data')
        #    cancel_btn = QtGui.QPushButton('Cancel')
        #    msgBox.addButton(use_saved_btn, QtGui.QMessageBox.YesRole)
        #    msgBox.addButton(use_current_btn, QtGui.QMessageBox.YesRole)
        #    msgBox.addButton(cancel_btn, QtGui.QMessageBox.NoRole)
        #    msgBox.exec_()

        #    if (msgBox.clickedButton() == use_current_btn):
        #        self.process_data()
        #        for p in self.plots:
        #            p.plot_slot()
        #        self.data_saved = False
        #    elif (msgBox.clickedButton() == use_saved_btn):
        #        self.batch_process()
        #    else:
        #        return


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
                self.save_hdf5_file(filename)

            elif (filetype == "*.csv"):
                if '.' not in filename:
                    filename += '.csv'
                self.save_csv_file(filename)
            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return False

            self.message_slot("data saved to  " + filename + "\n")
            self.data_saved = True
            return True


    def load_csv_file(self, filename):

        if (self.receiver.USE_ENCODER):
            expected_non_imu_columns = 2
        else:
            expected_non_imu_columns = 1

        self.receiver.num_imus = None
        #with open(filename, 'rb') as datafile:
        with open(filename, 'r') as datafile:
            reader = csv.reader(datafile, delimiter=',')
            for row in reader:
                if (len(row) % 9 != expected_non_imu_columns):
                    self.error_slot("invalid csv file\n")
                    return False
                row_num_imus = (len(row) - 2) // 9       # // for integer division in python3
                if self.receiver.num_imus is None:       # READING FIRST LINE OF CSV
                    self.receiver.num_imus = row_num_imus
                    self.receiver.reset_data(row_num_imus)
                else:
                    if (len(row) != (self.receiver.num_imus * 9) + expected_non_imu_columns):
                        self.error_slot("invalid csv file\n")
                        return False
                row = list(map(lambda x: float(x) if ('.' in x) else int(x), row))
                self.receiver.imu_data['timestamps'].append(row[0])

                j = 1
                for i in range(0, row_num_imus):
                    self.receiver.imu_data['imus'][i]['accel'][0].append(row[j])
                    self.receiver.imu_data['imus'][i]['accel'][1].append(row[j+1])
                    self.receiver.imu_data['imus'][i]['accel'][2].append(row[j+2])
                    self.receiver.imu_data['imus'][i]['gyro'][0].append(row[j+3])
                    self.receiver.imu_data['imus'][i]['gyro'][1].append(row[j+4])
                    self.receiver.imu_data['imus'][i]['gyro'][2].append(row[j+5])
                    self.receiver.imu_data['imus'][i]['mag'][0].append(row[j+6])
                    self.receiver.imu_data['imus'][i]['mag'][1].append(row[j+7])
                    self.receiver.imu_data['imus'][i]['mag'][2].append(row[j+8])
                    j+=9

                self.receiver.imu_data['encoder'].append(row[-1])
            return True
        return False



    def load_hdf5_file(self, filename):
        with h5py.File(filename, 'r') as datafile:
            self.receiver.imu_data = {}
            self.receiver.imu_data['timestamps'] = datafile.get('data/t')[()]
            self.receiver.imu_data['imus'] = []

            i = 0
            ext = ""
            while ('data/Accel' + ext in datafile and 'data/Gyro' + ext in datafile and 'data/Mag' + ext in datafile):
                self.receiver.imu_data['imus'].append({})
                self.receiver.imu_data['imus'][i]['accel'] = list(map(list, zip(*datafile.get('data/Accel' + ext)[()])))
                self.receiver.imu_data['imus'][i]['gyro'] = list(map(list, zip(*datafile.get('data/Gyro' + ext)[()])))
                self.receiver.imu_data['imus'][i]['mag'] = list(map(list, zip(*datafile.get('data/Mag' + ext)[()])))
                i += 1
                ext = str(i + 1)

            self.receiver.num_imus = i

            if (self.receiver.USE_ENCODER):
                self.receiver.imu_data['encoder'] = datafile.get('data/Encoder')[()]
            return True
        return False


    def save_hdf5_file(self, filename):
        with h5py.File(filename, 'w') as datafile:
            save_data = datafile.create_group("data")

            save_data.create_dataset('t', data=self.receiver.imu_data['timestamps'])
            for i in range(0, len(self.receiver.imu_data['imus'])):
                imu = self.receiver.imu_data['imus'][i]
                extension = "" if i < 1 else str(i + 1)

                save_data.create_dataset('Accel' + extension, data=list(zip(*self.receiver.imu_data['imus'][i]['accel'])))
                save_data.create_dataset('Gyro'  + extension, data=list(zip(*self.receiver.imu_data['imus'][i]['gyro'])))
                save_data.create_dataset('Mag'   + extension, data=list(zip(*self.receiver.imu_data['imus'][i]['mag'])))

            if (self.receiver.USE_ENCODER):
                save_data.create_dataset('Encoder', data=self.receiver.imu_data['encoder'])



    def save_csv_file(self, filename):
        with open(filename, 'w') as datafile:
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





    def load_button_slot(self):

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        filename, filetype = QFileDialog.getOpenFileName(self, "Choose a file", self.last_data_path, "*.hdf5;;*.csv", options=options)

        if filename:
            filename = str(filename)
            self.last_data_path = os.path.dirname(filename)

            if (filetype == "*.hdf5"):
                self.load_hdf5_file(filename)
            elif (filetype == "*.csv"):
                self.load_csv_file(filename)
            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return

            self.make_plots()
            for p in self.plots:
                p.plot_slot()

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
        self.buttons['save'].setEnabled(self.receiver.has_data())
        #self.buttons['save'].setEnabled(len(self.receiver.imu_data['timestamps']) > 0)
        #self.buttons['test'].setEnabled(True)



        # CROP DATA IF PRE-TRIGGER
        if (self.receiver.has_data()):
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
    #sys.exit(app.exec_())
    return app.exec_()
          
if __name__ == '__main__':
    main()
