#!/usr/bin/python

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from am_gui import Am_ui

class Am_settings(QWidget):

    def __init__(self, parent=None):

        ########################################
        #               SETUP                  #
        ########################################
        super(Am_settings, self).__init__(parent)
        self.parent = parent

        self.setWindowModality(Qt.ApplicationModal)
        self.setWindowTitle(Am_ui.APPLICATION_NAME) 

        top_layout = QGridLayout()
        top_layout.setColumnMinimumWidth(2, 50)

        
        ########################################
        #              COSNTANTS               #
        ########################################
        self.PRE_TRIGGER_MIN = 1
        self.PRE_TRIGGER_MAX = 180
        self.POST_TRIGGER_MIN = 1
        self.POST_TRIGGER_MAX = 180


        ########################################
        #  CHECKBOX TO ENABLE/DISABLE TRIGGER  #
        ########################################
        self.checkbox_trigger = QCheckBox('use trigger', self)
        self.checkbox_trigger.setChecked(self.parent.receiver.use_trigger)
        self.checkbox_trigger.stateChanged.connect(self.toggle_trigger)


        ########################################
        #       LABEL FOR TRIGGER DELAY        #
        ########################################
        self.pre_trigger_label = QLabel("pre-trigger delay (sec.)")
        self.pre_trigger_label.setEnabled(self.parent.receiver.use_trigger)


        ########################################
        #       SLIDER FOR TRIGGER DELAY       #
        ########################################
        self.slider_pre_trigger = QSlider(Qt.Horizontal, self)
        self.slider_pre_trigger.setValue(self.parent.pre_trigger_delay)
        self.slider_pre_trigger.setEnabled(self.parent.receiver.use_trigger)
        self.slider_pre_trigger.setFocusPolicy(Qt.NoFocus)
        self.slider_pre_trigger.valueChanged[int].connect(self.read_pre_trigger_slider_slot)
        self.slider_pre_trigger.setMinimum(self.PRE_TRIGGER_MIN)
        self.slider_pre_trigger.setMaximum(self.PRE_TRIGGER_MAX)


        ########################################
        #      TEXTBOX FOR TRIGGER DELAY       #
        ########################################
        self.textbox_pre_trigger = QLineEdit(str(self.parent.post_trigger_delay), self)
        validator = QIntValidator(self.PRE_TRIGGER_MIN, self.PRE_TRIGGER_MAX)
        self.textbox_pre_trigger.setValidator(validator)
        self.textbox_pre_trigger.setEnabled(self.parent.receiver.use_trigger)
        self.textbox_pre_trigger.editingFinished.connect(self.read_pre_trigger_text_slot)


        ########################################
        #      PLACE EVERYTHING IN LAYOUT      #
        ########################################
        top_layout.addWidget(self.checkbox_trigger, 1, 1)
        top_layout.addWidget(self.pre_trigger_label, 2, 1)
        top_layout.addWidget(self.slider_pre_trigger, 3, 1)
        top_layout.addWidget(self.textbox_pre_trigger, 3, 2)
        self.setLayout(top_layout)


    ########################################
    #        HANDLE TRIGGER CHECKBOX       #
    ########################################
    def toggle_trigger(self, state):
        if (state == Qt.Checked):
            self.slider_pre_trigger.setEnabled(True)
            self.textbox_pre_trigger.setEnabled(True)
            self.pre_trigger_label.setEnabled(True)
            self.parent.receiver.use_trigger = True
        else:
            self.slider_pre_trigger.setEnabled(False)
            self.textbox_pre_trigger.setEnabled(False)
            self.pre_trigger_label.setEnabled(False)
            self.parent.receiver.use_trigger = False

          
    ########################################
    #        HANDLE SLIDER CHANGES         #
    ########################################
    def read_pre_trigger_slider_slot(self, val):
        self.parent.pre_trigger_delay = val
        self.textbox_pre_trigger.setText(str(self.parent.pre_trigger_delay))


    ########################################
    #        HANDLE TEXTBOX ENTRY          #
    ########################################
    def read_pre_trigger_text_slot(self):
        val = int(self.textbox_pre_trigger.text())
        self.parent.pre_trigger_delay = val
        self.slider_pre_trigger.setValue(self.parent.pre_trigger_delay)


