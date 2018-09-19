import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging


from imucapture.data import Data
from imucapture.transform import Transform
from imucapture.global_data import *
from imucapture.transformed_data_window import Transformed_data_window

try:
    from PyQt5.QtCore import QString
except ImportError:
    QString = str


class Transform_dialog(PyQt5.QtWidgets.QWidget):

    finished_signal = PyQt5.QtCore.pyqtSignal()

    def __init__(self, data):


        ########################################
        #               SETUP                  #
        ########################################
        super().__init__()

        # SET WINDOW TITLE
        self.setWindowTitle(Global_data.APPLICATION_FULL_NAME) 

        self.data=data

        self.transform = Transform()

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        self.top_layout = PyQt5.QtWidgets.QGridLayout()

        self.process_algorithm = ""

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





        # BUTTONS
        button_layout = PyQt5.QtWidgets.QHBoxLayout()

        cancel_btn = PyQt5.QtWidgets.QPushButton('Cancel')
        cancel_btn.setToolTip('Cancel data processing')
        cancel_btn.clicked.connect(self.close)
        button_layout.addWidget(cancel_btn)

        self.transform_btn = PyQt5.QtWidgets.QPushButton('Transform')
        self.transform_btn.setToolTip('Apply the transformation, using the loaded calibration data, to the specified data')
        self.transform_btn.clicked.connect(self.transform_data)
        button_layout.addWidget(self.transform_btn)
 

        self.top_layout.addWidget(algorithm_box, 0, 0)
        self.top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(self.top_layout)






    def set_algorithm(self, name):
        if (name == 'madgwick' or name == 'integrate' or name == 'dsf'):
            self.process_algorithm = name



    def transform_data(self):
        logging.info("running " + str(self.process_algorithm))

        if(Global_data.calibration is None or Global_data.calibration.imu_bases is None):
            logging.error("can't transform data without basis vector from calibration file")
            return

        assert(Global_data.calibration is not None)

        if (self.data.has_data()):

            transformed_data = numpy.empty([self.data.num_imus, 3, 3, self.data.num_samples])

            for i in range(0, self.data.num_imus):

                if (self.process_algorithm == 'integrate'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_integrate(self.data, Global_data.calibration, i, num_filter_samples)

                elif (self.process_algorithm == 'madgwick'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_madgwick(self.data, Global_data.calibration, i, num_filter_samples)

                elif (self.process_algorithm == 'dsf'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_dsf(self.data, Global_data.calibration, i, num_filter_samples)

                else:
                    logging.error("invalid algorithm: " + self.process_algorithm)
                    return


                solution_mag = self.data.imu_data[i, Data.MAG_INDEX, :, :]

                if (solution_accel.shape != solution_gyro.shape or solution_accel.shape != solution_mag.shape):
                    logging.error("solution modalities don't have the same shape:")
                    logging.error('accel: ' + str(solution_accel.shape))
                    logging.error('gyro: ' + str(solution_gyro.shape))
                    logging.error('mag: ' + str(solution_mag.shape))
                    return


                transformed_data[i, :, :, :] = numpy.stack([solution_accel, solution_gyro, solution_mag])


            transformed_window = Transformed_data_window(Data.from_data('transformed', transformed_data))

            transformed_window.update()
            transformed_window.activate_buttons()
            transformed_window.show()

            self.close()







    # OVERRIDE
    def closeEvent(self, event):
        self.finished_signal.emit()
        event.accept()


