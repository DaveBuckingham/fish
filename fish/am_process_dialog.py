#!/usr/bin/python

from PyQt5.QtCore import *
from PyQt5.QtGui import *

class Am_process_dialog(QWidget):

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
        self.batch_output_filetupe = ""
        self.process_algorithm = ""

        self.filename_list = []

        
        ########################################
        #              COSNTANTS               #
        ########################################


        ########################################
        #          RADIO BUTOONS               #
        ########################################

        # Alex's approach to radio buttons was helpful:
        # http://stackoverflow.com/questions/17402452/how-to-get-the-checked-radiobutton-from-a-groupbox-in-pyqt

        top_layout = QGridLayout()

        # MODE RADIOS
        mode_layout = QVBoxLayout()
        self.mode_button_group = QButtonGroup(self)
        self.mode_radios = [QRadioButton("Use current data"), QRadioButton("Batch process")]

        for i in range(len(self.mode_radios)):
            mode_layout.addWidget(self.mode_radios[i])
            self.mode_button_group.addButton(self.mode_radios[i])


        # BATCH OPTIONS
        self.batch_layout = QVBoxLayout()
        select_files_btn = QPushButton('Select files')
        select_files_btn.clicked.connect(self.select_files)
        self.batch_layout.addWidget(select_files_btn)

        self.num_files_label = QLabel("0 files selected")
        self.batch_layout.addWidget(self.num_files_label)

        filetype_button_group = QButtonGroup(self)
        filetype_radios = [QRadioButton("hdf5"), QRadioButton("csv"), QRadioButton("input")]

        filetype_radios[0].clicked.connect(self.set_output_hdf5)
        filetype_radios[1].clicked.connect(self.set_output_csv)
        filetype_radios[2].clicked.connect(self.set_output_input)

        for i in range(len(filetype_radios)):
            self.batch_layout.addWidget(filetype_radios[i])
            filetype_button_group.addButton(filetype_radios[i])

        postfix_layout = QHBoxLayout()
        postfix_label = QLabel("output suffix:")
        postfix_layout.addWidget(postfix_label)

        postfix_textbox = QLineEdit("_processed")
        postfix_layout.addWidget(postfix_textbox)

        self.batch_layout.addLayout(postfix_layout)


        mode_layout.addLayout(self.batch_layout)


        self.mode_radios[0].clicked.connect(self.set_single_mode)
        self.mode_radios[1].clicked.connect(self.set_batch_mode)

        if(self.data.has_data()):
            self.mode_radios[0].click()
            #self.mode_radios[0].setChecked(True)
            #self.set_single_mode()
        else:
            self.mode_radios[0].setEnabled(False)
            self.mode_radios[1].click()
            #self.mode_radios[1].setChecked(True)
            #self.set_batch_mode()





        # ALGORITHM RADIOS
        algorithm_layout = QVBoxLayout()
        self.algorithm_button_group = QButtonGroup(self)
        self.algorithm_radios = [QRadioButton("Madgwick"), QRadioButton("Simple integration"), QRadioButton("Extended Kalman")]
        #self.algorithm_radios[0].setChecked(True)
        self.algorithm_radios[0].click()

        for i in range(len(self.algorithm_radios)):
            algorithm_layout.addWidget(self.algorithm_radios[i])
            self.algorithm_button_group.addButton(self.algorithm_radios[i])


        # BUTTONS
        button_layout = QHBoxLayout()

        cancel_btn = QPushButton('Cancel')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        process_btn = QPushButton('Process')
        process_btn.clicked.connect(self.run_process)
        button_layout.addWidget(process_btn)
 

        top_layout.addLayout(mode_layout, 0, 0)
        top_layout.addLayout(algorithm_layout, 0, 1)
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

    def set_algorith(self, name):
        if (name == 'madgwick' or name == 'integrate' or name == 'ekf'):
            self.process_algorithm = name


    def select_files(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        (self.filename_list, types) = QFileDialog.getOpenFileNames(self, "Select files to process", "", "*.hdf5 *.csv", options=options)
        #(filename_list, types) = QFileDialog.getOpenFileNames(self, "Select files to process", self.last_data_path, "*.hdf5 *.csv", options=options)
        self.num_files_label.setText('%d files selected' % len(self.filename_list))


    def process_current_dataset(self, algorithm):
        if (self.data.has_data()):
            for i in range(0, len(self.data.imu_data['timestamps'])):
                for j in range(0, len(self.data.imu_data['imus'])):
                    for k in range(0, 3):
                        self.data.imu_data['imus'][j]['accel'][k][i] *= -2
                        self.data.imu_data['imus'][j]['gyro'][k][i] *= -1
                        self.data.imu_data['imus'][j]['mag'][k][i] *= 2


    def process_multiple_datasets(self, algorithm):
        suffix = "_processed"

        if(self.filename_list):
            for filename in self.filename_list:
                (base, extension) = os.path.splitext(filename)
                if ((extension == ".hdf5") or (extension == ".csv")):

                    if (extension == ".hdf5"):
                        self.load_hdf5_file(filename)
                        self.process_current_dataset(algorithm)
                    elif (extension == ".csv"):
                        self.load_csv_file(filename)
                        self.process_current_dataset(algorithm)

                    if ((self.batch_output_filetype == "hdf5") or ((self.batch_output_filetype == "input") and (extension == ".hdf5"))):
                        out_name = base + suffix + ".hdf5"
                        self.save_hdf5_file(out_name)
                    elif ((self.batch_output_filetype == "csv") or ((self.batch_output_filetype == "input") and (extension == ".csv"))):
                        out_name = base + suffix + ".csv"
                        self.save_csv_file(out_name)

            self.data.reset_data(0)
            self.data_saved = True
            self.make_plots()


    def run_process(self):
        if(self.batch_process):
            self.process_multiple_datasets(self.process_algorithm)
        else:
            self.process_current_dataset(self.process_algorithm)

        self.close()


