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


class Am_ui(QWidget):

    APPLICATION_NAME = 'AquaMetric 0.01'

    plot_a1_signal = pyqtSignal(list)
    plot_a2_signal = pyqtSignal(list)
    plot_g1_signal = pyqtSignal(list)
    plot_g2_signal = pyqtSignal(list)
    
    clear_plots_signal = pyqtSignal()


    def __init__(self, parent = None):
        super(Am_ui, self).__init__(parent)

        self.data_saved = True
        self.recording = False

        self.use_trigger = False
        self.pre_trigger_delay = 10
        self.post_trigger_delay = 10

        self.buttons = {}

        top_layout = QGridLayout()

        # BUTTONS
        
        button_layout = QVBoxLayout()
        self.button_container = QWidget()
        self.button_container.setLayout(button_layout)

        self.buttons['test'] = QPushButton('Test')
        self.buttons['test'].setToolTip('Check communication with arduino and IMUs, run IMU self tests')
        button_layout.addWidget(self.buttons['test'])

        self.buttons['record'] = QPushButton('Record')
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['record'].clicked.connect(self.record_button_slot)
        button_layout.addWidget(self.buttons['record'])

        self.buttons['save'] = QPushButton('Save')
        self.buttons['save'].setToolTip('Save recorded data')
        self.buttons['save'].clicked.connect(self.save_button_slot)
        button_layout.addWidget(self.buttons['save'])
        self.buttons['save'].setEnabled(False)

        self.buttons['settings'] = QPushButton('Settings')
        self.buttons['settings'].clicked.connect(self.settings_button_slot)
        button_layout.addWidget(self.buttons['settings'])

        self.buttons['quit'] = QPushButton('Quit')
        self.buttons['quit'].clicked.connect(self.quit_button_slot)
        button_layout.addWidget(self.buttons['quit'])





        # TEXT WINDOW

        self.text_window = QTextEdit()
        self.text_window.setReadOnly(True)
        #metrics = QFontMetrics(self.text_window.font())


        # GRAPHS

        self.plot_a1 = Am_plot()
        self.plot_a2 = Am_plot()
        self.plot_g1 = Am_plot()
        self.plot_g2 = Am_plot()


        # STATS
        self.stats = QLabel("# Samples: \nTime: 37")
        self.stats.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignCenter)


        # ADD WIDGETS TO LAYOUT

        top_layout.addWidget(self.button_container, 1, 3, 2, 1)

        top_layout.addWidget(self.stats, 3, 3)

        top_layout.addWidget(self.plot_a1, 1, 1)
        top_layout.addWidget(self.plot_g1, 1, 2)
        top_layout.addWidget(self.plot_a2, 2, 1)
        top_layout.addWidget(self.plot_g2, 2, 2)

        top_layout.addWidget(self.text_window, 3, 1, 1, 2)
        

        # ADD LABELS
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


        # TOP LEVEL

        self.setLayout(top_layout)
        self.setWindowTitle(Am_ui.APPLICATION_NAME) 


        # RECEIVER THREAD

        self.receiver_thread = QThread()
        self.receiver = Am_rx()
        self.receiver.moveToThread(self.receiver_thread)


        # CONNECTIONS

        self.receiver.finished_signal.connect(self.receiver_thread.quit)
        self.receiver_thread.started.connect(self.receiver.run)
        self.receiver_thread.finished.connect(self.receiver_done)

        self.receiver.sample_signal.connect(self.sample_slot)
        self.receiver.message_signal.connect(self.message_slot)
        self.receiver.error_signal.connect(self.error_slot)

        self.plot_a1_signal.connect(self.plot_a1.data_slot)
        self.plot_a2_signal.connect(self.plot_a2.data_slot)
        self.plot_g1_signal.connect(self.plot_g1.data_slot)
        self.plot_a2_signal.connect(self.plot_g2.data_slot)

        self.clear_plots_signal.connect(self.plot_a1.clear_slot)
        self.clear_plots_signal.connect(self.plot_a2.clear_slot)
        self.clear_plots_signal.connect(self.plot_g1.clear_slot)
        self.clear_plots_signal.connect(self.plot_g2.clear_slot)


    def receiver_done(self):
        pass
        # do something?

    def sample_slot(self, values):
        self.plot_a1_signal.emit(values[2:5])
        self.plot_a2_signal.emit(values[5:8])
        self.plot_g1_signal.emit(values[8:11])
        self.plot_g2_signal.emit(values[11:14])



############################################
#              BUTTON SLOTS                #
############################################


    def quit_button_slot(self):
        result = (QMessageBox.question(self,
                                       Am_ui.APPLICATION_NAME,
                                       'Really quit?',
                                       QMessageBox.Yes | QMessageBox.No,
                                       QMessageBox.Yes))

        if (result == QMessageBox.Yes):
            self.stop_recording()
            self.message_slot("waiting for receiver to finish")
            self.receiver_thread.wait()
            self.message_slot("exiting")
            self.close()


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
                self.record()


    def save_button_slot(self):
        filename = QFileDialog.getSaveFileName(self, 'Save recorded data', '', '*.hdf5')
        if filename:
            if ( (len(filename) < 5) or (filename[-5:].toLower() != '.hdf5') ):
                filename += '.hdf5'
            self.message_slot("data saved to  " + filename)
            self.data_saved = True
            self.buttons['save'].setEnabled(False)

    def settings_button_slot(self):
        settings = Am_settings(self)
        settings.show()





    def record(self):
        self.recording = True
        self.receiver.recording = True
        self.data_saved = False

        if (self.use_trigger):
            self.buttons['record'].setText('Trigger')
        else:
            self.buttons['record'].setText('Stop')

        self.buttons['record'].setToolTip('Stop recording samples')

        self.buttons['save'].setEnabled(False)
        self.buttons['settings'].setEnabled(False)
        self.buttons['test'].setEnabled(False)

        self.clear_plots_signal.emit()

        self.receiver_thread.start()

    def stop_recording(self):
        self.recording = False
        self.receiver.recording = False
        self.buttons['record'].setText('Record')
        self.buttons['record'].setToolTip('Begin recording samples')
        self.buttons['save'].setEnabled(True)
        self.buttons['settings'].setEnabled(True)
        self.buttons['test'].setEnabled(True)





    def error_slot(self, the_string):
        self.message_slot(the_string, True)

    def message_slot(self, the_string, red=False):
        self.text_window.setTextColor(QtGui.QColor(120, 120, 120))
        self.text_window.insertPlainText("\n" + time.strftime("%c") + "    ")

        if (red):
            self.text_window.setTextColor(QtGui.QColor(255,0,0))
        else:
            self.text_window.setTextColor(QtGui.QColor(0,0,0))
        self.text_window.insertPlainText(the_string)
        sb = self.text_window.verticalScrollBar();
        sb.setValue(sb.maximum());

                                          
def main():
    app = QApplication(sys.argv)
    ex = Am_ui()
    ex.show()
    sys.exit(app.exec_())
          
if __name__ == '__main__':
    main()
