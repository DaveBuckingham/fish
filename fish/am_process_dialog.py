#!/usr/bin/python

import os
from PyQt5.QtCore import *
from PyQt5.QtGui import *

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str

class Am_process_dialog(QWidget):

    finished_signal = pyqtSignal()
    message_signal = pyqtSignal(QString)
    error_signal = pyqtSignal(QString)

    #def __init__(self, current_data=False, parent=None):
    def __init__(self, data):

        ########################################
        #               SETUP                  #
        ########################################
        super(Am_process_dialog, self).__init__()

        self.data=data

        self.setWindowModality(Qt.ApplicationModal)

        #self.setMaximumWidth(300)

        top_layout = QGridLayout()

        self.batch_process = False
        self.batch_output_filetype = ""
        self.process_algorithm = ""

        self.filename_list = []


        

        top_layout = QGridLayout()



        #########################################################
        #   BATCH OPTIONS, ACTIVE ONLY WHEN BATCH IS SELECTED   #
        #########################################################

        self.batch_layout = QVBoxLayout()

        # SELECT FILES BUTTON
        select_files_btn = QPushButton('Select files')
        select_files_btn.clicked.connect(self.select_files)
        self.batch_layout.addWidget(select_files_btn)

        # NUMBER OF FILES SELECTED
        self.num_files_label = QLabel("0 files selected")
        self.batch_layout.addWidget(self.num_files_label)

        # OUTPUT FILE TYPE
        filetype_box = QGroupBox("Output file type")
        filetype_layout = QVBoxLayout()

        radio = QRadioButton("hdf5")
        radio.clicked.connect(self.set_output_hdf5)
        filetype_layout.addWidget(radio)
        radio.click()

        radio = QRadioButton("csv")
        radio.clicked.connect(self.set_output_csv)
        filetype_layout.addWidget(radio)

        radio = QRadioButton("same as input")
        radio.clicked.connect(self.set_output_input)
        filetype_layout.addWidget(radio)

        filetype_box.setLayout(filetype_layout)
        self.batch_layout.addWidget(filetype_box)

        # OUTPUT FILENAME POSTFIX
        postfix_layout = QHBoxLayout()
        postfix_label = QLabel("output suffix:")
        postfix_layout.addWidget(postfix_label)

        self.postfix_textbox = QLineEdit("_processed")
        postfix_layout.addWidget(self.postfix_textbox)

        self.batch_layout.addLayout(postfix_layout)




        #########################################################
        #   CURRENT DATA OR BATCH MODE, INCLUDES BATCH OPTIONS  #
        #########################################################

        mode_layout = QVBoxLayout()
        mode_box = QGroupBox("Process mode")

        radio = QRadioButton("Use current data")
        radio.clicked.connect(self.set_single_mode)
        mode_layout.addWidget(radio);
        if(self.data.has_data()):
            radio.click()
        else:
            radio.setEnabled(False)

        radio = QRadioButton("Batch process")
        radio.clicked.connect(self.set_batch_mode)
        mode_layout.addWidget(radio)
        if(not self.data.has_data()):
            radio.click()

        mode_box.setLayout(mode_layout)



        #########################################################
        #          PROCESSIING ALGORITHM SELECTION              #
        #########################################################

        algorithm_layout = QVBoxLayout()
        algorithm_box = QGroupBox("Integration algorithm")


        radio = QRadioButton("Madgwick")
        radio.clicked.connect(lambda: self.set_algorithm('madgwick'))
        algorithm_layout.addWidget(radio)
        radio.click()

        radio = QRadioButton("Simple integration")
        radio.clicked.connect(lambda: self.set_algorithm('integrate'))
        algorithm_layout.addWidget(radio)

        radio = QRadioButton("Extended Kalman")
        radio.clicked.connect(lambda: self.set_algorithm('ekf'))
        algorithm_layout.addWidget(radio)

        algorithm_box.setLayout(algorithm_layout)





        # BUTTONS
        button_layout = QHBoxLayout()

        cancel_btn = QPushButton('Cancel')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        process_btn = QPushButton('Process')
        process_btn.clicked.connect(self.run_process)
        button_layout.addWidget(process_btn)
 

        top_layout.addWidget(mode_box, 0, 0)
        top_layout.addLayout(self.batch_layout, 1, 0)
        top_layout.addWidget(algorithm_box, 0, 1)
        top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(top_layout)


    # MAYBE MOVE THIS TO A TOOLS MODULE
    def enable_layout(self, layout, state):
        items = (layout.itemAt(i) for i in range(layout.count())) 
        for w in items:
            if isinstance(w, QLayout):
                self.enable_layout(w, state)
            else:
                w.widget().setEnabled(state)

    def set_single_mode(self):
        self.batch_process = False
        self.enable_layout(self.batch_layout, False)

    def set_batch_mode(self):
        self.batch_process = True
        self.enable_layout(self.batch_layout, True)

    def set_output_csv(self):
        self.batch_output_filetype = "csv"

    def set_output_hdf5(self):
        self.batch_output_filetype = "hdf5"

    def set_output_input(self):
        self.batch_output_filetype = "input"

    def set_algorithm(self, name):
        if (name == 'madgwick' or name == 'integrate' or name == 'ekf'):
            self.process_algorithm = name


    def select_files(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        (self.filename_list, types) = QFileDialog.getOpenFileNames(self, "Select files to process", "", "*.hdf5 *.csv", options=options)
        #(filename_list, types) = QFileDialog.getOpenFileNames(self, "Select files to process", self.last_data_path, "*.hdf5 *.csv", options=options)
        self.num_files_label.setText('%d files selected' % len(self.filename_list))


    def process_current_dataset(self, algorithm):
        self.message_signal.emit("running " + algorithm + "\n")
        if (self.data.has_data()):
            for i in range(0, len(self.data.imu_data['timestamps'])):
                for j in range(0, len(self.data.imu_data['imus'])):
                    for k in range(0, 3):
                        self.data.imu_data['imus'][j]['accel'][k][i] *= -2
                        self.data.imu_data['imus'][j]['gyro'][k][i] *= -1
                        self.data.imu_data['imus'][j]['mag'][k][i] *= 2


    def process_multiple_datasets(self, algorithm):
        suffix = self.postfix_textbox.text()

        if(self.filename_list):
            for filename in self.filename_list:
                (base, extension) = os.path.splitext(filename)
                if ((extension == ".hdf5") or (extension == ".csv")):

                    if (extension == ".hdf5"):
                        self.data.load_hdf5_file(filename)
                        self.process_current_dataset(algorithm)
                    elif (extension == ".csv"):
                        self.data.load_csv_file(filename)
                        self.process_current_dataset(algorithm)

                    if ((self.batch_output_filetype == "hdf5") or ((self.batch_output_filetype == "input") and (extension == ".hdf5"))):
                        out_name = base + suffix + ".hdf5"
                        self.data.save_hdf5_file(out_name)
                    elif ((self.batch_output_filetype == "csv") or ((self.batch_output_filetype == "input") and (extension == ".csv"))):
                        out_name = base + suffix + ".csv"
                        self.data.save_csv_file(out_name)

            self.data.reset_data(0)
            self.data.saved = True


    def run_process(self):
        if(self.batch_process):
            self.process_multiple_datasets(self.process_algorithm)
        else:
            self.process_current_dataset(self.process_algorithm)
        self.close()

    # OVERRIDE
    def closeEvent(self, event):
        self.finished_signal.emit()
        event.accept()


