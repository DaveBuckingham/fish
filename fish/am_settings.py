#!/usr/bin/python

from PyQt5.QtCore import *
from PyQt5.QtGui import *

class Am_settings(QWidget):

    def __init__(self, parent=None):

        
        self.use_trigger = False
        self.data_buffer_len = 600


        ########################################
        #               SETUP                  #
        ########################################
        super(Am_settings, self).__init__(parent)
        self.parent = parent

        self.setWindowModality(Qt.ApplicationModal)

        self.setMaximumWidth(300)

        top_layout = QGridLayout()

        
        ########################################
        #              COSNTANTS               #
        ########################################
        self.DATA_BUFFER_MIN = 1
        self.DATA_BUFFER_MAX = 20000


        ########################################
        #  CHECKBOX TO ENABLE/DISABLE TRIGGER  #
        ########################################
        trigger_checkbox = QCheckBox('use trigger', self)
        trigger_checkbox.stateChanged.connect(self.toggle_trigger)


        ########################################
        #        DATA BUFFER SETTINGS          #
        ########################################

        # LAYOUT
        buffer_layout = QGridLayout()

        # LABEL
        #buffer_label = QLabel("data buffer length (sec.)")
        buffer_label = QLabel("data buffer length (# samples)")
        buffer_layout.addWidget(buffer_label, 1, 1)

        # SLIDER
        self.buffer_slider = QSlider(Qt.Horizontal, self)
        self.buffer_slider.setValue(self.data_buffer_len)
        self.buffer_slider.valueChanged[int].connect(self.read_buffer_slider_slot)
        self.buffer_slider.setMinimum(self.DATA_BUFFER_MIN)
        self.buffer_slider.setMaximum(self.DATA_BUFFER_MAX)
        buffer_layout.addWidget(self.buffer_slider, 2, 1)

        # TEXTBOX
        self.buffer_textbox = QLineEdit(str(self.data_buffer_len), self)
        #self.buffer_textbox.setMaximumWidth(50)
        self.buffer_textbox.setFixedWidth(60)
        validator = QIntValidator(self.DATA_BUFFER_MIN, self.DATA_BUFFER_MAX)
        self.buffer_textbox.setValidator(validator)
        self.buffer_textbox.editingFinished.connect(self.read_buffer_text_slot)
        buffer_layout.addWidget(self.buffer_textbox, 2, 2)


        ########################################
        #      PLACE EVERYTHING IN LAYOUT      #
        ########################################
        top_layout.addWidget(trigger_checkbox, 1, 1)
        top_layout.addLayout(buffer_layout, 2, 1)
        self.setLayout(top_layout)


    ########################################
    #        HANDLE TRIGGER CHECKBOX       #
    ########################################
    def toggle_trigger(self, state):
        self.use_trigger = not self.use_trigger

          
    ########################################
    #        HANDLE SLIDER CHANGES         #
    ########################################
    def read_buffer_slider_slot(self, val):
        self.data_buffer_len = val
        self.buffer_textbox.setText(str(self.data_buffer_len))


    ########################################
    #        HANDLE TEXTBOX ENTRY          #
    ########################################
    def read_buffer_text_slot(self):
        val = int(self.buffer_textbox.text())
        self.data_buffer_len = val
        self.buffer_slider.setValue(self.data_buffer_len)


