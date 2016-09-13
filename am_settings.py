#!/usr/bin/python

import os
import sys
import time
import serial
import struct
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from am_gui import Am_ui

#class Am_settings(QDialog):
class Am_settings(QWidget):


    def __init__(self, parent=None):
        
	self.PRE_TRIGGER_MIN = 1
	self.PRE_TRIGGER_MAX = 180
	self.POST_TRIGGER_MIN = 1
	self.POST_TRIGGER_MAX = 60

        super(Am_settings, self).__init__(parent)

        self.parent = parent

        self.setWindowModality(Qt.ApplicationModal)
        self.setWindowTitle(Am_ui.APPLICATION_NAME) 

        top_layout = QGridLayout()
        top_layout.setColumnMinimumWidth(2, 50)


        self.cb_trigger = QCheckBox('trigger', self)
        self.cb_trigger.setChecked(self.parent.use_trigger)
        self.cb_trigger.stateChanged.connect(self.toggle_trigger)


        self.slider_pre_trigger = QSlider(Qt.Horizontal, self)
        self.slider_pre_trigger.setValue(self.parent.pre_trigger_delay)
        self.slider_pre_trigger.setEnabled(self.parent.use_trigger)
        self.slider_pre_trigger.setFocusPolicy(Qt.NoFocus)
        self.slider_pre_trigger.valueChanged[int].connect(self.change_pre_trigger_slot)
        self.slider_pre_trigger.setMinimum(self.PRE_TRIGGER_MIN)
        self.slider_pre_trigger.setMaximum(self.PRE_TRIGGER_MAX)


        self.slider_post_trigger = QSlider(Qt.Horizontal, self)
        self.slider_post_trigger.setValue(self.parent.post_trigger_delay)
        self.slider_post_trigger.setEnabled(self.parent.use_trigger)
        self.slider_post_trigger.setFocusPolicy(Qt.NoFocus)
        self.slider_post_trigger.valueChanged[int].connect(self.change_post_trigger_slot)
        self.slider_post_trigger.setMinimum(self.POST_TRIGGER_MIN)
        self.slider_post_trigger.setMaximum(self.POST_TRIGGER_MAX)


        #self.cb.setStyle(QStyleFactory.create("Cleanlooks"))

        self.text_pre_trigger = QLabel(str(self.parent.pre_trigger_delay) + " sec.")
        self.text_post_trigger = QLabel(str(self.parent.post_trigger_delay) + " sec.")


        top_layout.addWidget(self.cb_trigger, 1, 1)
        top_layout.addWidget(QLabel("pre-trigger delay"), 2, 1)
        top_layout.addWidget(self.slider_pre_trigger, 3, 1)
        top_layout.addWidget(QLabel("post-trigger delay"), 4, 1)
        top_layout.addWidget(self.slider_post_trigger, 5, 1)

        top_layout.addWidget(self.text_pre_trigger, 3, 2)
        top_layout.addWidget(self.text_post_trigger, 5, 2)



        self.setLayout(top_layout)


    def toggle_trigger(self, state):
        if (state == Qt.Checked):
            self.slider_pre_trigger.setEnabled(True)
            self.slider_post_trigger.setEnabled(True)
            self.parent.use_trigger = True
        else:
            self.slider_pre_trigger.setEnabled(False)
            self.slider_post_trigger.setEnabled(False)
            self.parent.use_trigger = False

          

    def change_post_trigger_slot(self, val):
        self.parent.post_trigger_delay = val
        self.text_post_trigger.setText(str(val) + " sec.")

    def change_pre_trigger_slot(self, val):
        self.parent.pre_trigger_delay = val
        self.text_pre_trigger.setText(str(val) + " sec.")


