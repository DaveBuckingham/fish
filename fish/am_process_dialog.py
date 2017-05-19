import os

import PyQt5.QtCore
import PyQt5.QtGui


from fish.am_data import Am_data
from fish.am_process import Am_process

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Am_process_dialog(PyQt5.QtGui.QWidget):

    finished_signal = PyQt5.QtCore.pyqtSignal()
    message_signal = PyQt5.QtCore.pyqtSignal(QString)
    error_signal = PyQt5.QtCore.pyqtSignal(QString)


    #def __init__(self, current_data=False, parent=None):
    def __init__(self, data):

        ########################################
        #               SETUP                  #
        ########################################
        super(Am_process_dialog, self).__init__()

        self.data=data

        self.process = Am_process()

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        #self.setMaximumWidth(300)

        top_layout = PyQt5.QtGui.QGridLayout()

        self.batch_process = False
        self.batch_output_filetype = ""
        self.process_algorithm = ""

        self.filename_list = []

        

        top_layout = PyQt5.QtGui.QGridLayout()



        #########################################################
        #   BATCH OPTIONS, ACTIVE ONLY WHEN BATCH IS SELECTED   #
        #########################################################

        self.batch_layout = PyQt5.QtGui.QVBoxLayout()

        # SELECT FILES BUTTON
        select_files_btn = PyQt5.QtGui.QPushButton('Select input files')
        select_files_btn.clicked.connect(self.select_files)
        self.batch_layout.addWidget(select_files_btn)

        # NUMBER OF FILES SELECTED
        self.num_files_label = PyQt5.QtGui.QLabel("0 files selected")
        self.batch_layout.addWidget(self.num_files_label)

        # OUTPUT FILE TYPE
        filetype_box = PyQt5.QtGui.QGroupBox("Output file type")
        filetype_layout = PyQt5.QtGui.QVBoxLayout()

        radio = PyQt5.QtGui.QRadioButton("hdf5")
        radio.clicked.connect(self.set_output_hdf5)
        filetype_layout.addWidget(radio)
        radio.click()

        radio = PyQt5.QtGui.QRadioButton("csv")
        radio.clicked.connect(self.set_output_csv)
        filetype_layout.addWidget(radio)

        radio = PyQt5.QtGui.QRadioButton("same as input")
        radio.clicked.connect(self.set_output_input)
        filetype_layout.addWidget(radio)

        filetype_box.setLayout(filetype_layout)
        self.batch_layout.addWidget(filetype_box)

        # OUTPUT FILENAME POSTFIX
        postfix_layout = PyQt5.QtGui.QHBoxLayout()
        postfix_label = PyQt5.QtGui.QLabel("output suffix:")
        postfix_layout.addWidget(postfix_label)

        self.postfix_textbox = PyQt5.QtGui.QLineEdit("_processed")
        postfix_layout.addWidget(self.postfix_textbox)

        self.batch_layout.addLayout(postfix_layout)




        #########################################################
        #   CURRENT DATA OR BATCH MODE, INCLUDES BATCH OPTIONS  #
        #########################################################

        mode_layout = PyQt5.QtGui.QVBoxLayout()
        mode_box = PyQt5.QtGui.QGroupBox("Process mode")

        radio = PyQt5.QtGui.QRadioButton("Use current data")
        radio.clicked.connect(self.set_single_mode)
        mode_layout.addWidget(radio);
        if(self.data.has_data()):
            radio.click()
        else:
            radio.setEnabled(False)

        radio = PyQt5.QtGui.QRadioButton("Batch process")
        radio.clicked.connect(self.set_batch_mode)
        mode_layout.addWidget(radio)
        if(not self.data.has_data()):
            radio.click()

        mode_box.setLayout(mode_layout)



        #########################################################
        #          PROCESSIING ALGORITHM SELECTION              #
        #########################################################

        algorithm_layout = PyQt5.QtGui.QVBoxLayout()
        algorithm_box = PyQt5.QtGui.QGroupBox("Integration algorithm")


        radio = PyQt5.QtGui.QRadioButton("Madgwick")
        radio.clicked.connect(lambda: self.set_algorithm('madgwick'))
        algorithm_layout.addWidget(radio)
        radio.click()

        radio = PyQt5.QtGui.QRadioButton("Simple integration")
        radio.clicked.connect(lambda: self.set_algorithm('integrate'))
        algorithm_layout.addWidget(radio)

        radio = PyQt5.QtGui.QRadioButton("Extended Kalman")
        radio.clicked.connect(lambda: self.set_algorithm('ekf'))
        algorithm_layout.addWidget(radio)

        algorithm_box.setLayout(algorithm_layout)


        #########################################################
        #                     CALIBRATION SETUP                 #
        #########################################################

        calibration_layout = PyQt5.QtGui.QVBoxLayout()
        select_calibration_btn = PyQt5.QtGui.QPushButton('Parse calibration file')
        select_calibration_btn.clicked.connect(self.parse_calibration)
        calibration_layout.addWidget(select_calibration_btn)



        # BUTTONS
        button_layout = PyQt5.QtGui.QHBoxLayout()

        cancel_btn = PyQt5.QtGui.QPushButton('Cancel')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        process_btn = PyQt5.QtGui.QPushButton('Process')
        process_btn.clicked.connect(self.run_process)
        button_layout.addWidget(process_btn)
 

        top_layout.addWidget(algorithm_box, 0, 0)
        top_layout.addLayout(calibration_layout, 1, 0)
        top_layout.addWidget(mode_box, 0, 1)
        top_layout.addLayout(self.batch_layout, 1, 1)
        top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(top_layout)


    # MAYBE MOVE THIS TO A TOOLS MODULE
    def enable_layout(self, layout, state):
        items = (layout.itemAt(i) for i in range(layout.count())) 
        for w in items:
            if isinstance(w, PyQt5.QtGui.QLayout):
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
        options = PyQt5.QtGui.QFileDialog.Options()
        options |= PyQt5.QtGui.QFileDialog.DontUseNativeDialog
        (self.filename_list, types) = PyQt5.QtGui.QFileDialog.getOpenFileNames(self, "Select files to process", "", "*.hdf5 *.csv", options=options)
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

    def parse_calibration(self):

        calib_data = Am_data()

        # THIS CODE COPIED FROM AM_GUI, THERE MUST BE A BETTER WAY
        options = PyQt5.QtGui.QFileDialog.Options()
        options |= PyQt5.QtGui.QFileDialog.DontUseNativeDialog
        filename, filetype = PyQt5.QtGui.QFileDialog.getOpenFileName(self, "Choose a file", filter="*.hdf5;;*.csv", options=options)

        if filename:
            filename = str(filename)

            if (filetype == "*.hdf5"):
                calib_data.load_hdf5_file(filename)
            elif (filetype == "*.csv"):
                calib_data.load_csv_file(filename)
            else:
                self.error_slot("invalid file type: " + filetype + "\n")
                return

        vals = self.process.get_calib_values(calib_data)
        print("VALS:")
        for v in vals:
            print(v)


        

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


