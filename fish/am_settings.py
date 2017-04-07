#!/usr/bin/python

import PyQt5.QtCore
import PyQt5.QtGui

class Am_settings(PyQt5.QtGui.QWidget):

    def __init__(self, parent=None):


        ########################################
        #              COSNTANTS               #
        ########################################
        self.DATA_BUFFER_MIN = 1                  # >0
        self.DATA_BUFFER_MAX = 10000

        
        self.use_trigger = False
        self.invert_trigger = False
        self.data_buffer_len = int((self.DATA_BUFFER_MAX - self.DATA_BUFFER_MIN + 1) / 2)


        ########################################
        #               SETUP                  #
        ########################################
        super(Am_settings, self).__init__(parent)
        self.parent = parent

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        self.setMaximumWidth(300)


        


        ########################################
        #  CHECKBOX TO ENABLE/DISABLE TRIGGER  #
        ########################################
        trigger_checkbox = PyQt5.QtGui.QCheckBox('use trigger', self)
        trigger_checkbox.stateChanged.connect(self.toggle_trigger)

        self.invert_checkbox = PyQt5.QtGui.QCheckBox('invert trigger', self)
        self.invert_checkbox.setToolTip('Trigger activates on pin low')
        self.invert_checkbox.stateChanged.connect(self.toggle_invert)
        #self.invert_checkbox.setEnabled(False)


        ########################################
        #        DATA BUFFER SETTINGS          #
        ########################################

        # LAYOUT
        buffer_layout = PyQt5.QtGui.QGridLayout()

        # LABEL
        #buffer_label = PyQt5.QtGui.QLabel("data buffer length (sec.)")
        buffer_label = PyQt5.QtGui.QLabel("data buffer length (# samples)")
        buffer_layout.addWidget(buffer_label, 1, 1)

        # SLIDER
        self.buffer_slider = PyQt5.QtGui.QSlider(PyQt5.QtCore.Qt.Horizontal, self)
        self.buffer_slider.setMinimum(self.DATA_BUFFER_MIN)
        self.buffer_slider.setMaximum(self.DATA_BUFFER_MAX)
        self.buffer_slider.setValue(self.data_buffer_len)
        self.buffer_slider.valueChanged[int].connect(self.read_buffer_slider_slot)
        buffer_layout.addWidget(self.buffer_slider, 2, 1)

        # TEXTBOX
        self.buffer_textbox = PyQt5.QtGui.QLineEdit(str(self.data_buffer_len), self)
        #self.buffer_textbox.setMaximumWidth(50)
        self.buffer_textbox.setFixedWidth(60)
        validator = PyQt5.QtGui.QIntValidator(self.DATA_BUFFER_MIN, self.DATA_BUFFER_MAX)
        self.buffer_textbox.setValidator(validator)
        self.buffer_textbox.editingFinished.connect(self.read_buffer_text_slot)
        buffer_layout.addWidget(self.buffer_textbox, 2, 2)


        trigger_layout = PyQt5.QtGui.QHBoxLayout()
        trigger_layout.addWidget(trigger_checkbox)
        trigger_layout.addWidget(self.invert_checkbox)


        ########################################
        #      PLACE EVERYTHING IN LAYOUT      #
        ########################################
        top_layout = PyQt5.QtGui.QVBoxLayout()
        top_layout.addLayout(trigger_layout)
        top_layout.addLayout(buffer_layout)
        self.setLayout(top_layout)


    ########################################
    #        HANDLE TRIGGER CHECKBOX       #
    ########################################
    def toggle_trigger(self, state):
        self.use_trigger = not self.use_trigger
        #self.invert_checkbox.setEnabled(self.use_trigger)

    def toggle_invert(self, state):
        self.invert_trigger = not self.invert_trigger

          
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


