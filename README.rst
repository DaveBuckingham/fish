
INSTALLATION
============

PyQt4
-----
In additino to the packages listed in requirements.txt, which
are automatically installed by pip, it is necessary to install
PyQt4. It is not possible to install PyQt4 with pip.

**Windows** binaries can be download from riverbankcomputing:
``https://www.riverbankcomputing.com/software/pyqt/download``

**Debian** users can use apt-get to install PyQt4:
``apt-get install python-qt4``
or
``apt-get install python3-qt4``

**Build** it yourself.
First you need SIP:
http://www.riverbankcomputing.com/software/sip/download.
Then PyQt4:
http://www.riverbankcomputing.com/software/pyqt/download.




FILES
=====

am_gui.py
main program.
includes am_plot.py, am_rx.py, am_settings.py

am_plot.py
module for plotting data.
i.e. one of the 4 plot windows.

am_rx.py
module for com with arduino.

am_settings.py
module for program settings.

am_tx
directory containing my arduino code and the timer library it uses.

fish_rx.py
this was the first program i wrote for reading data from the arduino.
writes plain text to stdout.
not used by gui.
