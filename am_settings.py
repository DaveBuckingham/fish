#!/usr/bin/python

import os
import sys
import time
import serial
import struct
import random
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from am_gui import Am_ui

class Am_settings(QDialog):


    def __init__(self, parent=None):
        super(Am_settings, self).__init__(parent)

        self.parent = parent

        self.setWindowModality(Qt.ApplicationModal)
        self.setWindowTitle(Am_ui.APPLICATION_NAME) 

        top_layout = QGridLayout()


        self.cb_post = QCheckBox('post trigger', self)
        self.cb_post.stateChanged.connect(self.toggle_post)


        self.slider_time = QSlider(Qt.Horizontal, self)
        self.slider_time.setFocusPolicy(Qt.NoFocus)
        self.slider_time.valueChanged[int].connect(self.change_time_slot)
        self.slider_time.setMinimum(1)
        self.slider_time.setMaximum(60)
        self.slider_time.setValue(self.parent.post_trigger_time)

        if (self.parent.post_trigger):
            self.cb_post.setChecked(True)
        else:
            self.slider_time.setEnabled(False)


        #self.cb.setStyle(QStyleFactory.create("Cleanlooks"))

        top_layout.addWidget(self.cb_post, 1, 1)
        top_layout.addWidget(self.slider_time, 2, 1)


        self.setLayout(top_layout)



    def toggle_post(self, state):
        self.parent.post_trigger = not self.parent.post_trigger
        self.slider_time.setEnabled(self.parent.post_trigger)

    def change_time_slot(self, val):
        self.parent.post_trigger_time = val    # EMIT
        print val
          



