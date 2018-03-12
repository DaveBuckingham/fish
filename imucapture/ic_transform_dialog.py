import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

#from pprint import pprint

from imucapture.ic_data import Ic_data
from imucapture.ic_calib import Ic_calib
from imucapture.ic_transform import Ic_transform
from imucapture.ic_global import *
from imucapture.ic_transformed_data_window import Ic_transformed_data_window

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Ic_transform_dialog(PyQt5.QtWidgets.QWidget):

    finished_signal = PyQt5.QtCore.pyqtSignal()

    def __init__(self, data):


        ########################################
        #               SETUP                  #
        ########################################
        super().__init__()

        self.calib = Ic_calib()

        # SET WINDOW TITLE
        self.setWindowTitle(Ic_global.APPLICATION_FULL_NAME) 

        self.data=data

        self.transform = Ic_transform()

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        #self.basis_vector = None
        #self.imu_bases = None

        #self.setMaximumWidth(300)

        self.top_layout = PyQt5.QtWidgets.QGridLayout()

        self.batch_process = False
        self.batch_output_filetype = ""
        self.process_algorithm = ""

        self.filename_list = []


        

        self.top_layout = PyQt5.QtWidgets.QGridLayout()





        #########################################################
        #          PROCESSIING ALGORITHM SELECTION              #
        #########################################################

        algorithm_layout = PyQt5.QtWidgets.QVBoxLayout()
        algorithm_box = PyQt5.QtWidgets.QGroupBox("Integration algorithm")

        radio = PyQt5.QtWidgets.QRadioButton("DSF")
        radio.clicked.connect(lambda: self.set_algorithm('dsf'))
        radio.setToolTip('Use the Dynamic Snap Free method')
        algorithm_layout.addWidget(radio)
        # DSF IS DEFAULT
        radio.click()
        self.set_algorithm('dsf')

        radio = PyQt5.QtWidgets.QRadioButton("Madgwick")
        radio.clicked.connect(lambda: self.set_algorithm('madgwick'))
        radio.setToolTip('Use the Madgwick (2010) orientation filter')
        algorithm_layout.addWidget(radio)

        radio = PyQt5.QtWidgets.QRadioButton("Simple integration")
        radio.clicked.connect(lambda: self.set_algorithm('integrate'))
        radio.setToolTip('Naively integrate over time; the Madgwick filter with beta=0')
        algorithm_layout.addWidget(radio)

        algorithm_box.setLayout(algorithm_layout)


        #########################################################
        #                     CALIBRATION SETUP                 #
        #########################################################

        calibration_layout = PyQt5.QtWidgets.QVBoxLayout()
        select_calibration_btn = PyQt5.QtWidgets.QPushButton('Parse calibration file')
        select_calibration_btn.setToolTip('Select and parse a calibration file')
        select_calibration_btn.clicked.connect(self.calibrate)
        calibration_layout.addWidget(select_calibration_btn)


        self.range_labels = []
        self.radio_layout = PyQt5.QtWidgets.QGridLayout()
        self.axis_select_matrix = []
        for i in range(0,3):
            self.axis_select_matrix.append([None, None, None])

        for i in range(0, 3):
            self.range_labels.append(PyQt5.QtWidgets.QLabel())
            self.radio_layout.addWidget(self.range_labels[i], i, 0)
            for j in range(0, 3):
                radio = PyQt5.QtWidgets.QRadioButton(('x','y','z')[j])
                #radio.setToolTip('')
                radio.clicked.connect(self.axis_radio_event)
                self.radio_layout.addWidget(radio, i, j+1)
                self.axis_select_matrix[i][j] = radio
        calibration_layout.addLayout(self.radio_layout)
        Ic_global.enable_layout(self.radio_layout, False)

        for i in range(0,3):
            for j in range(0,3):
                if (i == j):
                    self.axis_select_matrix[i][j].setChecked(True)
                else:
                    self.axis_select_matrix[i][j].setChecked(False)



        # BUTTONS
        button_layout = PyQt5.QtWidgets.QHBoxLayout()

        cancel_btn = PyQt5.QtWidgets.QPushButton('Cancel')
        cancel_btn.setToolTip('Cancel data processing')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        self.transform_btn = PyQt5.QtWidgets.QPushButton('Transform')
        self.transform_btn.setToolTip('Apply the transformation, using the loaded calibration file, to the specified data')
        self.transform_btn.clicked.connect(self.transform_data)
        button_layout.addWidget(self.transform_btn)
        self.transform_btn.setEnabled(False)
 

        self.top_layout.addWidget(algorithm_box, 0, 0)
        self.top_layout.addLayout(calibration_layout, 1, 0)
        self.top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(self.top_layout)

    def calibrate(self):
        if (self.calib.parse_calibration()):
            self.range_labels[0].setText(str(self.calib.intervals[0]))
            self.range_labels[1].setText(str(self.calib.intervals[1]))
            self.range_labels[2].setText(str(self.calib.intervals[2]))
            Ic_global.enable_layout(self.radio_layout, True)
            self.transform_btn.setEnabled(True)


    # MAINTAIN THE AXIS SELECTION RADIOS AS A PERMUTATION MATRIX
    # I.E. ONE RADIO SELECTED PER ROW AND PER COLUMN
    # THERE MUST BE A BETTER WAY TO DO THIS EG WITH LINEAR ALGEBRA
    def axis_radio_event(self):
        # FIND THE ROW AND COLUMN WITH TWO SELECTED RADIOS
        rows = [0,1,2]
        cols = [0,1,2]
        double_row = None
        double_col = None
        for i in range(0,3):
            for j in range(0,3):
                if self.axis_select_matrix[i][j].isChecked():
                    if (i in rows):
                        rows.remove(i)
                    else:
                        double_row = i
                    if (j in cols):
                        cols.remove(j)
                    else:
                        double_col = j

        # DESELECT ONE RADIO FROM EACH, NOT THE INTERSECTION
        if ((double_row is not None) and (double_col is not None)):
            for i in range(0,3):
                if (i != double_row):
                    self.axis_select_matrix[i][double_col].setChecked(False)
                if (i != double_col):
                    self.axis_select_matrix[double_row][i].setChecked(False) 

        # FILL IN THE RADIO AT A ROW AND COLUMN WITH NO CHECKED RADIOS
        empty_row = [0,1,2]
        empty_col = [0,1,2]
        for i in range(0,3):
            for j in range(0,3):
                if self.axis_select_matrix[i][j].isChecked():
                    empty_row.remove(i)
                    empty_col.remove(j)
        self.axis_select_matrix[empty_row[0]][empty_col[0]].setChecked(True)



    def set_axes(self, button):
        id = self.axis_buttons.id(button)
        row = id // 3
        col = id % 3
        print('row: ' + str(row))
        print('col: ' + str(col))


    # MAYBE MOVE THIS TO A TOOLS MODULE
    def enable_layout(self, layout, state):
        items = (layout.itemAt(i) for i in range(layout.count())) 
        for w in items:
            if isinstance(w, PyQt5.QtWidgets.QLayout):
                self.enable_layout(w, state)
            else:
                w.widget().setEnabled(state)



    def set_algorithm(self, name):
        if (name == 'madgwick' or name == 'integrate' or name == 'dsf'):
            self.process_algorithm = name



    def transform_data(self):
        logging.info("running " + str(self.process_algorithm) + "\n")

        if(self.calib.imu_bases is None):
            logging.error("can't transform data without basis vector from calibration file")
            return


        if (self.data.has_data()):

            boolean_axis_select = numpy.matrix([[self.axis_select_matrix[j][k].isChecked() for j in range(3)] for k in range(3)])
            for i in range(0, self.data.num_imus):

                # ADJUST CALIB AXES ACCORDING TO USER SETTINGS
                self.calib.imu_bases[i] = numpy.array(numpy.dot(boolean_axis_select, self.calib.imu_bases[i]))

                if (self.process_algorithm == 'integrate'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_integrate(self.data, self.calib, i, num_filter_samples)


                elif (self.process_algorithm == 'madgwick'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_madgwick(self.data, self.calib, i, num_filter_samples)

                elif (self.process_algorithm == 'dsf'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_dsf(self.data, self.calib, i, num_filter_samples)

                
                else:
                    logging.error("invalid algorithm: " + self.process_algorithm)
                    return

                assert(self.data.num_samples == len(solution_accel))
                assert(self.data.num_samples == len(solution_gyro))


            transformed_data = Ic_data()
            transformed_data.reset_data(self.data.num_imus)

            for sample_index in range(0, self.data.num_samples):
                sample = []
                for imu in range(0, self.data.num_imus):
                    sample.append([solution_accel[sample_index], solution_gyro[sample_index], [0, 0, 0]])
                    # MAYBE WE SHOULD COPY OVER THE MAG FROM THE RAW DATA

                transformed_data.add_sample(sample)

            transformed_window = Ic_transformed_data_window(transformed_data)
            transformed_window.update()
            transformed_window.activate_buttons()
            transformed_window.show()

            self.close()












    # OVERRIDE
    def closeEvent(self, event):
        self.finished_signal.emit()
        event.accept()


