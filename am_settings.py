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

        if (parent.post_trigger):
            self.cb_post.toggle()

        #self.cb.setStyle(QStyleFactory.create("Cleanlooks"))

        top_layout.addWidget(self.cb_post, 1, 1)


        self.setLayout(top_layout)


        #cb.move(20, 20)
        #cb.toggle()
        self.cb_post.stateChanged.connect(self.changeTitle)

    def changeTitle(self, state):
        self.parent.post_trigger = not self.parent.post_trigger
          
#        if state == Qt.Checked:
#            self.setWindowTitle('QtGui.QCheckBox')
#        else:
#            self.setWindowTitle('')






