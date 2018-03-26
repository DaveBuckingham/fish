import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging

from imucapture.ic_global import *


class Ic_calib_dialog(PyQt5.QtWidgets.QWidget):

    def __init__(self, calib):


        ########################################
        #               SETUP                  #
        ########################################
        super().__init__()


        # SET WINDOW TITLE
        self.setWindowTitle("Specify calibration axes")

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        self.top_layout = PyQt5.QtWidgets.QGridLayout()

        self.calib = calib





        radio_layout = PyQt5.QtWidgets.QGridLayout()
        self.button_matrix = []
        for i in range(0,3):
            self.button_matrix.append([None, None, None])

        for i in range(0, 3):
            range_as_seconds = (calib.intervals[i][0] / Ic_global.SAMPLE_FREQ_HZ, calib.intervals[i][1] / Ic_global.SAMPLE_FREQ_HZ)
            range_label = PyQt5.QtWidgets.QLabel(str(range_as_seconds))
            radio_layout.addWidget(range_label, i, 0)
            for j in range(0, 3):
                radio = PyQt5.QtWidgets.QRadioButton(('x','y','z')[j])
                radio.setAutoExclusive(False)
                radio.setToolTip('')
                radio.clicked.connect(self.axis_radio_event)
                radio_layout.addWidget(radio, i, j+1)
                self.button_matrix[i][j] = radio

        for i in range(0,3):
            for j in range(0,3):
                if (i == j):
                    self.button_matrix[i][j].setChecked(True)
                else:
                    self.button_matrix[i][j].setChecked(False)



        # BUTTONS
        button_layout = PyQt5.QtWidgets.QHBoxLayout()

        cancel_btn = PyQt5.QtWidgets.QPushButton('Cancel')
        cancel_btn.setToolTip('Cancel data processing')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        self.transform_btn = PyQt5.QtWidgets.QPushButton('Apply')
        self.transform_btn.setToolTip('Set the selected axes')
        self.transform_btn.clicked.connect(self.set_axes)
        button_layout.addWidget(self.transform_btn)
 

        self.top_layout.addLayout(radio_layout, 1, 0)
        self.top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(self.top_layout)


    # MAINTAIN RADIO BUTTON STATE AS A PERMUTATION MATRIX
    def axis_radio_event(self):
        matrix = numpy.array([[button.isChecked() for button in row] for row in self.button_matrix])

        if (matrix.sum() < 3) :
            empty_col = matrix.sum(0).tolist().index(0)
            empty_row = matrix.sum(1).tolist().index(0)
            matrix[empty_row, empty_col] = True

            for (row, col), value in numpy.ndenumerate(matrix):
                self.button_matrix[row][col].setChecked(value)

        elif (matrix.sum() > 3) :

            double_col = matrix.sum(0).tolist().index(2)
            double_row = matrix.sum(1).tolist().index(2)
            matrix[double_row, :] = False
            matrix[:, double_col] = False
            matrix[double_row, double_col] = True

            empty_col = matrix.sum(0).tolist().index(0)
            empty_row = matrix.sum(1).tolist().index(0)
            matrix[empty_row, empty_col] = True

            for (row, col), value in numpy.ndenumerate(matrix):
                self.button_matrix[row][col].setChecked(value)




    def set_axes(self):
        boolean_axis_select = numpy.matrix([[self.button_matrix[j][k].isChecked() for j in range(3)] for k in range(3)])
        for i in range(0, len(self.calib.imu_bases)):
            self.calib.imu_bases[i] = numpy.array(numpy.dot(boolean_axis_select, self.calib.imu_bases[i]))
        Ic_global.calibration = self.calib
        logging.info("calibration set")
        for window in (Ic_global.data_window_list):
            window.activate_buttons()
        self.close()





