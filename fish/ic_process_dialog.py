import os

import numpy
import PyQt5.QtCore
import PyQt5.QtGui
import logging

from fish.ic_data import Ic_data
from fish.ic_get_basis import Ic_get_basis
from fish.ic_process import Ic_process

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_process_dialog(PyQt5.QtGui.QWidget):

    finished_signal = PyQt5.QtCore.pyqtSignal()


    #def __init__(self, current_data=False, parent=None):
    def __init__(self, data):

        ########################################
        #               SETUP                  #
        ########################################
        super(Ic_process_dialog, self).__init__()

        self.data=data

        self.get_basis = Ic_get_basis()

        self.process = Ic_process()

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        self.basis_vector = None

        #self.setMaximumWidth(300)

        top_layout = PyQt5.QtGui.QGridLayout()

        self.batch_process = False
        self.batch_output_filetype = ""
        self.process_algorithm = ""

        self.filename_list = []

        self.calib_initial_gravity = None
        self.calib_still_accel = None
        self.calib_still_gyro = None

        

        top_layout = PyQt5.QtGui.QGridLayout()



        #########################################################
        #   BATCH OPTIONS, ACTIVE ONLY WHEN BATCH IS SELECTED   #
        #########################################################

        self.batch_layout = PyQt5.QtGui.QVBoxLayout()

        # SELECT FILES BUTTON
        select_files_btn = PyQt5.QtGui.QPushButton('Select input files')
        select_files_btn.setToolTip('Select files for batch processing')
        select_files_btn.clicked.connect(self.select_files)
        self.batch_layout.addWidget(select_files_btn)

        # NUMBER OF FILES SELECTED
        self.num_files_label = PyQt5.QtGui.QLabel("0 files selected")
        self.batch_layout.addWidget(self.num_files_label)

        # OUTPUT FILE TYPE
        filetype_box = PyQt5.QtGui.QGroupBox("Output file type")
        filetype_layout = PyQt5.QtGui.QVBoxLayout()

        radio = PyQt5.QtGui.QRadioButton("hdf5")
        radio.setToolTip('Batch process outputs files in hdf5 format')
        radio.clicked.connect(self.set_output_hdf5)
        filetype_layout.addWidget(radio)
        radio.click()

        radio = PyQt5.QtGui.QRadioButton("csv")
        radio.setToolTip('Batch process outputs files in csv format')
        radio.clicked.connect(self.set_output_csv)
        filetype_layout.addWidget(radio)

        radio = PyQt5.QtGui.QRadioButton("same as input")
        radio.setToolTip('Each batch process output file uses the same format as the corresponding input file')
        radio.clicked.connect(self.set_output_input)
        filetype_layout.addWidget(radio)

        filetype_box.setLayout(filetype_layout)
        self.batch_layout.addWidget(filetype_box)

        # OUTPUT FILENAME POSTFIX
        postfix_layout = PyQt5.QtGui.QHBoxLayout()
        postfix_label = PyQt5.QtGui.QLabel("output suffix:")
        postfix_label.setToolTip('Suffix appended to input file names to construct batch processing output file names')
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
        radio.setToolTip('Apply the filter to the current data buffer')
        radio.clicked.connect(self.set_single_mode)
        mode_layout.addWidget(radio);
        if(self.data.has_data()):
            radio.click()
        else:
            radio.setEnabled(False)

        radio = PyQt5.QtGui.QRadioButton("Batch process")
        radio.setToolTip('Apply the filter to one or more saved files')
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

        radio = PyQt5.QtGui.QRadioButton("DSF")
        radio.clicked.connect(lambda: self.set_algorithm('dsf'))
        radio.setToolTip('Use the Dynamic Snap Free method')
        algorithm_layout.addWidget(radio)
        radio.click()  # OUR'S IS THE BEST...

        radio = PyQt5.QtGui.QRadioButton("Madgwick")
        radio.clicked.connect(lambda: self.set_algorithm('madgwick'))
        radio.setToolTip('Use the Madgwick (2010) orientation filter')
        algorithm_layout.addWidget(radio)

        radio = PyQt5.QtGui.QRadioButton("Simple integration")
        radio.clicked.connect(lambda: self.set_algorithm('integrate'))
        radio.setToolTip('Naively integrate over time; the Madgwick filter with beta=0')
        algorithm_layout.addWidget(radio)

        algorithm_box.setLayout(algorithm_layout)


        #########################################################
        #                     CALIBRATION SETUP                 #
        #########################################################

        calibration_layout = PyQt5.QtGui.QVBoxLayout()
        select_calibration_btn = PyQt5.QtGui.QPushButton('Parse calibration file')
        select_calibration_btn.setToolTip('Select and parse a calibration file')
        select_calibration_btn.clicked.connect(self.parse_calibration)
        calibration_layout.addWidget(select_calibration_btn)



        # BUTTONS
        button_layout = PyQt5.QtGui.QHBoxLayout()

        cancel_btn = PyQt5.QtGui.QPushButton('Cancel')
        cancel_btn.setToolTip('Cancel data processing')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        self.process_btn = PyQt5.QtGui.QPushButton('Process')
        self.process_btn.setToolTip('Apply the filter, using the loaded calibration file, to the specified data')
        self.process_btn.clicked.connect(self.run_process)
        button_layout.addWidget(self.process_btn)
        self.process_btn.setEnabled(False)
 

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
        if (name == 'madgwick' or name == 'integrate' or name == 'dsf'):
            self.process_algorithm = name


    def select_files(self):
        options = PyQt5.QtGui.QFileDialog.Options()
        options |= PyQt5.QtGui.QFileDialog.DontUseNativeDialog
        (self.filename_list, searchtypes) = PyQt5.QtGui.QFileDialog.getOpenFileNames(self, "Select files to process", "", "*.csv *.hdf5", options=options)
        self.num_files_label.setText('%d files selected' % len(self.filename_list))


    def process_current_dataset(self, algorithm):
        # CANE WE REMOVE THIS?
        logging.info("running " + algorithm + "\n")

        if (self.data.has_data()):

            for i in range(0, self.data.num_imus):

                if (algorithm == 'integrate'):
                    filter_samples = 10
                    (solution_accel, solution_gyro) = self.process.get_orientation_integrate(self.basis_vector, self.data, filter_samples)

                elif (algorithm == 'madgwick'):
                    filter_samples = 10
                    (solution_accel, solution_gyro) = self.process.get_orientation_madgwick(self.basis_vector, self.data, filter_samples)

                elif (algorithm == 'dsf'):
                    filter_samples = 10
                    ca = (1.0, 1.0, 1.0)
                    (solution_accel, solution_gyro) = self.process.get_orientation_dsf(ca, self.calib_initial_gravity, self.basis_vector, self.calib_still_accel, self.calib_still_gyro, self.data, filter_samples)

                
                else:
                    logging.error("invalid algorithm: " + algorithm)
                    return

                assert(len(self.data.imu_data['timestamps']) == len(solution_accel))
                assert(len(self.data.imu_data['timestamps']) == len(solution_gyro))


                for j in range(0, len(self.data.imu_data['timestamps'])):
                    for k in range(0, 3):
                        self.data.imu_data['imus'][i]['accel'][k][j] = solution_accel[j][k]
                        self.data.imu_data['imus'][i]['gyro'][k][j]  = solution_gyro[j][k]



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

        calib_data = Ic_data()

        options = PyQt5.QtGui.QFileDialog.Options() | PyQt5.QtGui.QFileDialog.DontUseNativeDialog
        filename, searchtype = PyQt5.QtGui.QFileDialog.getOpenFileName(self, "Choose a file", filter="*.csv *.hdf5", options=options)

        if filename:
            filename = str(filename)
            prefix, extension = os.path.splitext(filename)

            logging.info("loading " + filename)

            if (extension == "*.hdf5"):
                if not calib_data.load_hdf5_file(filename):
                    logging.error("load failed\n")
                    return
            elif (extension == "*.csv"):
                if not calib_data.load_csv_file(filename):
                    logging.error("load failed\n")
                    return
            else:
                logging.error("invalid file extension: " + extension + "\n")
                return
        else:
            return

        intervals = self.get_basis.get_intervals(calib_data)
        basis_vector = self.get_basis.get_basis_vector(calib_data, intervals)


        if basis_vector is not None:
            basis_vector = numpy.array(basis_vector)
            logging.info("extracted basis vector:\n" + str(basis_vector[0]) + "\n" + str(basis_vector[1]) + "\n" + str(basis_vector[2]))
            self.basis_vector = basis_vector
            
            still_start = intervals[0][0]
            still_end = intervals[0][1]

            calib_accel = calib_data.as_list_of_triples(0, 'accel')
            calib_gyro = calib_data.as_list_of_triples(0, 'gyro')

            self.calib_initial_gravity = numpy.mean(calib_accel[still_start:still_end], axis=0)
            self.calib_still_accel = numpy.array(calib_accel[still_start:still_end])
            self.calib_still_gyro = numpy.array(calib_gyro[still_start:still_end])

            self.process_btn.setEnabled(True)
        else:
            logging.error("failed to extract basis vector")






    def run_process(self):
        if(self.basis_vector is not None):
            if(self.batch_process):
                self.process_multiple_datasets(self.process_algorithm)
            else:
                self.process_current_dataset(self.process_algorithm)
            self.close()
        else:
            logging.error("can't process data without basis vector from calibration file")

    # OVERRIDE
    def closeEvent(self, event):
        self.finished_signal.emit()
        event.accept()


