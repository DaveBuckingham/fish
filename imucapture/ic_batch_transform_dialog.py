import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

from imucapture.ic_transform_dialog import Ic_transform_dialog

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_batch_transform_dialog(Ic_transform_dialog):

    def __init__(self):

        super().__init__(None)

        self.batch_output_filetype = ""

        self.filename_list = []



        self.batch_layout = PyQt5.QtWidgets.QVBoxLayout()

        # SELECT FILES BUTTON
        select_files_btn = PyQt5.QtWidgets.QPushButton('Select input files')
        select_files_btn.setToolTip('Select files for batch processing')
        select_files_btn.clicked.connect(self.select_files)
        self.batch_layout.addWidget(select_files_btn)

        # NUMBER OF FILES SELECTED
        self.num_files_label = PyQt5.QtWidgets.QLabel("0 files selected")
        self.batch_layout.addWidget(self.num_files_label)

        # OUTPUT FILE TYPE
        filetype_box = PyQt5.QtWidgets.QGroupBox("Output file type")
        filetype_layout = PyQt5.QtWidgets.QVBoxLayout()

        radio = PyQt5.QtWidgets.QRadioButton("hdf5")
        radio.setToolTip('Batch process outputs files in hdf5 format')
        radio.clicked.connect(self.set_output_hdf5)
        filetype_layout.addWidget(radio)
        radio.click()

        radio = PyQt5.QtWidgets.QRadioButton("csv")
        radio.setToolTip('Batch process outputs files in csv format')
        radio.clicked.connect(self.set_output_csv)
        filetype_layout.addWidget(radio)

        radio = PyQt5.QtWidgets.QRadioButton("same as input")
        radio.setToolTip('Each batch process output file uses the same format as the corresponding input file')
        radio.clicked.connect(self.set_output_input)
        filetype_layout.addWidget(radio)

        filetype_box.setLayout(filetype_layout)
        self.batch_layout.addWidget(filetype_box)

        # OUTPUT FILENAME POSTFIX
        postfix_layout = PyQt5.QtWidgets.QHBoxLayout()
        postfix_label = PyQt5.QtWidgets.QLabel("output suffix:")
        postfix_label.setToolTip('Suffix appended to input file names to construct batch processing output file names')
        postfix_layout.addWidget(postfix_label)

        self.postfix_textbox = PyQt5.QtWidgets.QLineEdit("_processed")
        postfix_layout.addWidget(self.postfix_textbox)

        self.batch_layout.addLayout(postfix_layout)

        self.top_layout.addLayout(self.batch_layout, 0, 1)



    def set_output_csv(self):
        self.batch_output_filetype = "csv"

    def set_output_hdf5(self):
        self.batch_output_filetype = "hdf5"

    def set_output_input(self):
        self.batch_output_filetype = "input"



    def select_files(self):
        options = PyQt5.QtWidgets.QFileDialog.Options()
        options |= PyQt5.QtWidgets.QFileDialog.DontUseNativeDialog
        (self.filename_list, searchtypes) = PyQt5.QtWidgets.QFileDialog.getOpenFileNames(parent=self,
                                                                                     caption="Select files to process",
                                                                                     directory=Ic_global.last_file_path,
                                                                                     filter="*.csv *.hdf5",
                                                                                     options=options)
        self.num_files_label.setText('%d files selected' % len(self.filename_list))

        # STORE SOMETHING IN Ic_globa.last_file_path ???




    def transform_multiple_datasets(self, algorithm):

        if(self.basis_vector is None):
            logging.error("can't transform data without basis vector from calibration file")
            return

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



