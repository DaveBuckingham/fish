import os

import numpy
import PyQt5.QtCore
import PyQt5.QtWidgets
import logging


from imucapture.ic_data import Ic_data
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

        # SET WINDOW TITLE
        self.setWindowTitle(Ic_global.APPLICATION_FULL_NAME) 

        self.data=data

        self.transform = Ic_transform()

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

        if(Ic_global.calibration is None or Ic_global.calibration.imu_bases is None):
            logging.error("can't transform data without basis vector from calibration file")
            return


        if (self.data.has_data()):

            for i in range(0, self.data.num_imus):

                if (self.process_algorithm == 'integrate'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_integrate(self.data, Ic_global.calibration, i, num_filter_samples)

                elif (self.process_algorithm == 'madgwick'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_madgwick(self.data, Ic_global.calibration, i, num_filter_samples)

                elif (self.process_algorithm == 'dsf'):
                    num_filter_samples = 10
                    (solution_accel, solution_gyro) = self.transform.get_orientation_dsf(self.data, Ic_global.calibration, i, num_filter_samples)

                else:
                    logging.error("invalid algorithm: " + self.process_algorithm)
                    return

                assert(self.data.num_samples == len(solution_accel))
                assert(self.data.num_samples == len(solution_gyro))


            transformed_data = Ic_data("transformed")
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


