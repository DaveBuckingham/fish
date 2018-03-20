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





        self.range_labels = []
        radio_layout = PyQt5.QtWidgets.QGridLayout()
        self.axis_select_matrix = []
        for i in range(0,3):
            self.axis_select_matrix.append([None, None, None])

        for i in range(0, 3):
            self.range_labels.append(PyQt5.QtWidgets.QLabel())
            radio_layout.addWidget(self.range_labels[i], i, 0)
            for j in range(0, 3):
                radio = PyQt5.QtWidgets.QRadioButton(('x','y','z')[j])
                #radio.setToolTip('')
                radio.clicked.connect(self.axis_radio_event)
                radio_layout.addWidget(radio, i, j+1)
                self.axis_select_matrix[i][j] = radio

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

        self.transform_btn = PyQt5.QtWidgets.QPushButton('Apply')
        self.transform_btn.setToolTip('Set the selected axes')
        self.transform_btn.clicked.connect(self.set_axes)
        button_layout.addWidget(self.transform_btn)
        self.transform_btn.setEnabled(False)
 

        self.top_layout.addLayout(radio_layout, 1, 0)
        self.top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(self.top_layout)



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




    def set_axes(self):
        boolean_axis_select = numpy.matrix([[self.axis_select_matrix[j][k].isChecked() for j in range(3)] for k in range(3)])
        for i in range(0, len(calib.imu_bases)):
            Ic_global.calib.imu_bases[i] = numpy.array(numpy.dot(boolean_axis_select, calib.imu_bases[i]))
        logging.info("calibration set")
        self.close





