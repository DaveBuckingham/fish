#! /usr/bin/env python

import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


class Am_ui(QWidget):
    def __init__(self, parent = None):
        super(Am_ui, self).__init__(parent)

        self.data_saved = False
        self.recording = False
                     
        self.button_widget = QWidget()

        button_layout = QVBoxLayout()
        top_layout = QGridLayout()

        self.button_record = QPushButton('Record')
        self.button_record.setToolTip('Begin recording samples')
        self.button_record.clicked.connect(self.process_record_button)
        button_layout.addWidget(self.button_record)

        self.button_save = QPushButton('Save')
        self.button_save.setToolTip('Save recorded data')
        self.button_save.clicked.connect(self.save)
        button_layout.addWidget(self.button_save)

        self.button_widget.setLayout(button_layout)

        top_layout.addWidget(self.button_widget)
                     


        self.setLayout(top_layout)

        self.setWindowTitle('AquaMetric') 
                     

    def process_record_button(self):
        if (self.recording):
            self.recording = False
            self.stop_recording()
            self.button_record.setText('Record')
            self.button_record.setToolTip('Begin recording samples')

        else:
            result = (QMessageBox.question(self,
                                           'Message',
                                           'Overwrite recorded data without saving?',
                                           QMessageBox.Yes | QMessageBox.No,
                                           QMessageBox.No))
            if (result == QMessageBox.Yes):
                self.recording = True
                self.button_record.setText('Stop')
                self.button_record.setToolTip('Stop recording samples')
                self.record()

    def record(self):
        self.data_saved = False

    def stop_recording(self):
        self.button_save.setEnabled(True)


    def save(self):
        filename = QFileDialog.getSaveFileName(self, 'Save recorded data', '', '*.hdf5')
        if filename:
            if ( (len(filename) < 5) or (filename[-5:].toLower() != '.hdf5') ):
                filename += '.hdf5'
            print filename
            data_saved = True
            self.button_save.setEnabled(False)

                                          
def main():
    app = QApplication(sys.argv)
    ex = Am_ui()
    ex.show()
    sys.exit(app.exec_())
          
if __name__ == '__main__':
    main()
