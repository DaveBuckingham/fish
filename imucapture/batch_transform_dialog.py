import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

from imucapture.transform_dialog import Transform_dialog

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Batch_transform_dialog(Transform_dialog):

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

        # OUTPUT FILENAME POSTFIX
        suffix_label = PyQt5.QtWidgets.QLabel("output suffix:")
        suffix_label.setToolTip('Suffix appended to input file names to construct batch processing output file names')
        suffix_layout.addWidget(suffix_label)

        self.suffix_textbox = PyQt5.QtWidgets.QLineEdit("_processed")
        suffix_layout.addWidget(self.suffix_textbox)

        self.batch_layout.addLayout(suffix_layout)

        self.top_layout.addLayout(self.batch_layout, 0, 1)





    def select_files(self):
        options = PyQt5.QtWidgets.QFileDialog.Options()
        options |= PyQt5.QtWidgets.QFileDialog.DontUseNativeDialog
        (self.filename_list, searchtypes) = PyQt5.QtWidgets.QFileDialog.getOpenFileNames(parent=self,
                                                                                     caption="Select files to process",
                                                                                     filter="*.hdf5",
                                                                                     options=options)
        self.num_files_label.setText('%d files selected' % len(self.filename_list))





    def transform_multiple_datasets(self, algorithm):

        if(self.basis_vector is None):
            logging.error("can't transform data without basis vector from calibration file")
            return

        suffix = self.suffix_textbox.text()

        if(self.filename_list):
            for filename in self.filename_list:
                (base, extension) = os.path.splitext(filename)
                if (extension == ".hdf5"):
                    self.data.load_hdf5_file(filename)
                    self.process_current_dataset(algorithm)

                    out_name = base + suffix + ".hdf5"
                    self.data.save_hdf5_file(out_name)

            self.data.reset_data(0)



