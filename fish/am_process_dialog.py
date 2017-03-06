#!/usr/bin/python

from PyQt5.QtCore import *
from PyQt5.QtGui import *

class Am_process_dialog(QWidget):

    def __init__(self, current_data=False, parent=None):

        ########################################
        #               SETUP                  #
        ########################################
        super(Am_process_dialog, self).__init__(parent)
        self.parent = parent

        self.setWindowModality(Qt.ApplicationModal)

        #self.setMaximumWidth(300)

        top_layout = QGridLayout()

        
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

        if(current_data):
            self.mode_radios[0].setChecked(True)
        else:
            self.mode_radios[0].setEnabled(False)
            self.mode_radios[1].setChecked(True)

        for i in xrange(len(self.mode_radios)):
            mode_layout.addWidget(self.mode_radios[i])
            self.mode_button_group.addButton(self.mode_radios[i])


        # BATCH OPTIONS
        self.batch_layout = QVBoxLayout()
        select_files_btn = QPushButton('Select files')
        self.batch_layout.addWidget(select_files_btn)

        num_files_label = QLabel("0 files selected")
        #num_files_label.setText('Samples: %d' % num_samples)
        #self.pre_trigger_label.setEnabled(self.parent.receiver.use_trigger)
        self.batch_layout.addWidget(num_files_label)

        filetype_button_group = QButtonGroup(self)
        filetype_radios = [QRadioButton("hdf5"), QRadioButton("csv")]

        for i in xrange(len(filetype_radios)):
            self.batch_layout.addWidget(filetype_radios[i])
            filetype_button_group.addButton(filetype_radios[i])

        postfix_layout = QHBoxLayout()
        postfix_label = QLabel("output suffix:")
        postfix_layout.addWidget(postfix_label)

        postfix_textbox = QLineEdit("_processed")
        postfix_layout.addWidget(postfix_textbox)

        self.batch_layout.addLayout(postfix_layout)

        self.enable_layout(self.batch_layout, False)

        mode_layout.addLayout(self.batch_layout)


        # ALGORITHM RADIOS
        algorithm_layout = QVBoxLayout()
        self.algorithm_button_group = QButtonGroup(self)
        self.algorithm_radios = [QRadioButton("Madgwick"), QRadioButton("Simple integration"), QRadioButton("Extended Kalman")]
        self.algorithm_radios[0].setChecked(True)

        for i in xrange(len(self.algorithm_radios)):
            algorithm_layout.addWidget(self.algorithm_radios[i])
            self.algorithm_button_group.addButton(self.algorithm_radios[i])


        # BUTTONS
        button_layout = QHBoxLayout()
        cancel_btn = QPushButton('Cancel')
        process_btn = QPushButton('Process')
        button_layout.addWidget(cancel_btn)
        button_layout.addWidget(process_btn)
 

        top_layout.addLayout(mode_layout, 0, 0)
        top_layout.addLayout(algorithm_layout, 0, 1)
        top_layout.addLayout(button_layout, 2, 0, 2, 0)
        self.setLayout(top_layout)


    def enable_layout(self, layout, state):
        items = (layout.itemAt(i) for i in range(layout.count())) 
        for w in items:
            if isinstance(w, QLayout):
                self.enable_layout(w, state)
            else:
                w.widget().setEnabled(state)

    



