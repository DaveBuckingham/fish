#!/usr/bin/python

import PyQt5.QtCore
import PyQt5.QtWidgets
import PyQt5.QtGui

from imucapture.global_data import Global_data
from imucapture.txrx import Txrx

class Settings(PyQt5.QtWidgets.QWidget):

    data_buffer_length_signal = PyQt5.QtCore.pyqtSignal(int)

    def __init__(self, parent=None):


        ########################################
        #              VARIABLES               #
        ########################################
        self.data_buffer_len = int((Global_data.DATA_BUFFER_MAX - Global_data.DATA_BUFFER_MIN + 1) / 2)
        self.trigger_delay = int((Global_data.TRIGGER_DELAY_MAX - Global_data.TRIGGER_DELAY_MIN + 1) / 2)


        ########################################
        #               SETUP                  #
        ########################################
        super(Settings, self).__init__(parent)
        self.parent = parent

        self.setWindowModality(PyQt5.QtCore.Qt.ApplicationModal)

        self.setMaximumWidth(300)



        ########################################
        #        TRIGGER DELAY SETTINGS        #
        ########################################

        self.trigger_delay_layout = PyQt5.QtWidgets.QVBoxLayout()

        # LABEL
        trigger_delay_label = PyQt5.QtWidgets.QLabel("trigger delay")
        self.trigger_delay_layout.addWidget(trigger_delay_label)

        # SLIDER
        self.trigger_delay_slider = PyQt5.QtWidgets.QSlider(PyQt5.QtCore.Qt.Horizontal, self)
        self.trigger_delay_slider.setMinimum(Global_data.TRIGGER_DELAY_MIN)
        self.trigger_delay_slider.setMaximum(Global_data.TRIGGER_DELAY_MAX)
        self.trigger_delay_slider.setValue(self.trigger_delay)
        self.trigger_delay_slider.valueChanged[int].connect(self.read_trigger_delay_slider_slot)
        self.trigger_delay_slider.setToolTip('Set how many samples to collect after trigger')
        #trigger_layout.addWidget(self.trigger_delay_slider, 2, 2, PyQt5.QtCore.Qt.AlignRight)
        self.trigger_delay_layout.addWidget(self.trigger_delay_slider)


        # TEXTBOX
        trigger_textbox_layout = PyQt5.QtWidgets.QHBoxLayout()
        self.trigger_delay_textbox = PyQt5.QtWidgets.QLineEdit(str(self.trigger_delay), self)
        self.trigger_delay_textbox.setFixedWidth(60)
        validator = PyQt5.QtGui.QIntValidator(Global_data.TRIGGER_DELAY_MIN, Global_data.TRIGGER_DELAY_MAX)
        self.trigger_delay_textbox.setValidator(validator)
        self.trigger_delay_textbox.editingFinished.connect(self.read_trigger_delay_text_slot)
        trigger_textbox_layout.addWidget(self.trigger_delay_textbox)

        # SECONDS
        self.trigger_delay_label_sec = PyQt5.QtWidgets.QLabel("samples = " + str(Global_data.TRIGGER_DELAY_MAX * Global_data.MS_PER_SAMPLE) + " ms")
        self.trigger_delay_label_sec.setMinimumSize(self.trigger_delay_label_sec.sizeHint())
        self.trigger_delay_label_sec.setText('samples = ' + str(self.trigger_delay * Global_data.MS_PER_SAMPLE) + ' ms')
        trigger_textbox_layout.addWidget(self.trigger_delay_label_sec)

        #trigger_layout.addLayout(trigger_textbox_layout, 3, 1, 1, 2)
        self.trigger_delay_layout.addLayout(trigger_textbox_layout)






           
        ########################################
        #        TRIGGER RADIO BUTTONS         #
        ########################################

        trigger_radio_layout = PyQt5.QtWidgets.QVBoxLayout()


        radio = PyQt5.QtWidgets.QRadioButton("no trigger")
        radio.setToolTip('Trigger is ignored')
        radio.clicked.connect(self.set_trigger_off)
        trigger_radio_layout.addWidget(radio)
        radio.click()

        radio = PyQt5.QtWidgets.QRadioButton("rising edge")
        radio.setToolTip('Trigger is activated by a rising edge')
        radio.clicked.connect(self.set_trigger_rising_edge)
        trigger_radio_layout.addWidget(radio)

        radio = PyQt5.QtWidgets.QRadioButton("falling edge")
        radio.setToolTip('Trigger is activated by a falling edge')
        radio.clicked.connect(self.set_trigger_falling_edge)
        trigger_radio_layout.addWidget(radio)

        # ?????
        trigger_edge_box = PyQt5.QtWidgets.QGroupBox()
        trigger_edge_box.setLayout(trigger_radio_layout)





        ########################################
        #        DATA BUFFER SETTINGS          #
        ########################################

        # LAYOUT
        data_buffer_layout = PyQt5.QtWidgets.QVBoxLayout()


        # LABEL
        data_buffer_label = PyQt5.QtWidgets.QLabel("data buffer size")
        data_buffer_layout.addWidget(data_buffer_label)

        # SLIDER
        self.data_buffer_slider = PyQt5.QtWidgets.QSlider(PyQt5.QtCore.Qt.Horizontal, self)
        self.data_buffer_slider.setMinimum(Global_data.DATA_BUFFER_MIN)
        self.data_buffer_slider.setMaximum(Global_data.DATA_BUFFER_MAX)
        self.data_buffer_slider.setValue(self.data_buffer_len)
        self.data_buffer_slider.valueChanged[int].connect(self.read_data_buffer_slider_slot)
        self.data_buffer_slider.setToolTip('Set how many measurements the data buffer can hold')
        data_buffer_layout.addWidget(self.data_buffer_slider)

        data_buffer_textbox_layout = PyQt5.QtWidgets.QHBoxLayout()

        # TEXTBOX
        self.data_buffer_textbox = PyQt5.QtWidgets.QLineEdit(str(self.data_buffer_len), self)
        self.data_buffer_textbox.setFixedWidth(60)
        validator = PyQt5.QtGui.QIntValidator(Global_data.DATA_BUFFER_MIN, Global_data.DATA_BUFFER_MAX)
        self.data_buffer_textbox.setValidator(validator)
        self.data_buffer_textbox.editingFinished.connect(self.read_data_buffer_text_slot)
        data_buffer_textbox_layout.addWidget(self.data_buffer_textbox)

        # SECONDS
        self.data_buffer_label_sec = PyQt5.QtWidgets.QLabel("samples = " + str(Global_data.DATA_BUFFER_MAX * Global_data.MS_PER_SAMPLE) + " ms")
        self.data_buffer_label_sec.setMinimumSize(self.data_buffer_label_sec.sizeHint())
        self.data_buffer_label_sec.setText('samples = ' + str(self.data_buffer_len * Global_data.MS_PER_SAMPLE) + ' ms')
        data_buffer_textbox_layout.addWidget(self.data_buffer_label_sec)

        data_buffer_layout.addLayout(data_buffer_textbox_layout)


        ########################################
        #      PLACE EVERYTHING IN LAYOUT      #
        ########################################
        top_layout = PyQt5.QtWidgets.QVBoxLayout()
        top_layout.addWidget(trigger_edge_box)
        top_layout.addLayout(self.trigger_delay_layout)
        top_layout.addLayout(data_buffer_layout)
        self.setLayout(top_layout)


    ###################################################
    #   HELPER FUNCTION: ENABLE OR DISABLE A LAYOUT   #
    ###################################################
    def enable_layout(self, layout, state):
        items = (layout.itemAt(i) for i in range(layout.count())) 
        for w in items:
            if isinstance(w, PyQt5.QtWidgets.QLayout):
                self.enable_layout(w, state)
            else:
                w.widget().setEnabled(state)


    ########################################
    #        HANDLE TRIGGER RADIOS         #
    ########################################

    def set_trigger_off(self):
        self.use_trigger = Txrx.NO_TRIGGER_EDGE
        self.enable_layout(self.trigger_delay_layout, False)

    def set_trigger_rising_edge(self):
        self.use_trigger = Txrx.RISING_TRIGGER_EDGE
        self.enable_layout(self.trigger_delay_layout, True)

    def set_trigger_falling_edge(self):
        self.use_trigger = Txrx.FALLING_TRIGGER_EDGE
        self.enable_layout(self.trigger_delay_layout, True)


    ########################################
    #       HANDLE TRIGGER SLIDER CHANGES  #
    ########################################
    def read_trigger_delay_slider_slot(self, val):
        self.trigger_delay = val
        self.trigger_delay_textbox.setText(str(self.trigger_delay))
        # SHOULD READ FREQ FROM GLOBAL!
        self.trigger_delay_label_sec.setText('samples = ' + str(self.trigger_delay * 5) + ' ms')


    ########################################
    #       HANDLE TRIGGER TEXTBOX ENTRY   #
    ########################################
    def read_trigger_delay_text_slot(self):
        val = int(self.trigger_delay_textbox.text())
        self.trigger_delay = val
        self.trigger_delay_slider.setValue(self.trigger_delay)
        self.trigger_delay_label_sec.setText('samples = ' + str(self.trigger_delay * 5) + ' ms')

          
    ########################################
    #        HANDLE BUFFER SLIDER CHANGES  #
    ########################################
    def read_data_buffer_slider_slot(self, val):
        self.data_buffer_len = val
        self.data_buffer_textbox.setText(str(self.data_buffer_len))
        # SHOULD READ FREQ FROM GLOBAL!
        self.data_buffer_label_sec.setText('samples = ' + str(self.data_buffer_len * 5) + ' ms')
        self.data_buffer_length_signal.emit(self.data_buffer_len)


    ########################################
    #        HANDLE BUFFER TEXTBOX ENTRY   #
    ########################################
    def read_data_buffer_text_slot(self):
        val = int(self.data_buffer_textbox.text())
        self.data_buffer_len = val
        self.data_buffer_slider.setValue(self.data_buffer_len)
        self.data_buffer_label_sec.setText('samples = ' + str(self.data_buffer_len * 5) + ' ms')
        self.data_buffer_length_signal.emit(self.data_buffer_len)


