
INSTALLATION
============

PyQt4
-----

It is first necessary to install PyQt4. It is *not* possible to use
pip to install PyQt4.

Windows
    Binaries can be download from riverbankcomputing:
        https://www.riverbankcomputing.com/software/pyqt/download

Debian
    Python2
        ``# apt-get install python-qt4``
    Python3
        ``# apt-get install python3-qt4``

Source
    You can build PyQt yourself.
        First you need SIP:
            http://www.riverbankcomputing.com/software/sip/download
        Then PyQt4:
            http://www.riverbankcomputing.com/software/pyqt/download

pip
---

Once PyQt4 is installed, use pip to install *program_name*.

It may be necessary to update pip:
    ``# pip install --upgrade pip``

Then:
    ``# pip install`` *program_name*

Pip will automatically install dependencies other than PyQt4, which are listed in  ``requirements.txt``.


OPERATION
=========



FILES
=====

am_gui.py
    main program

am_plot.py
    module for plotting data i.e. one of the 4 plot windows

am_rx.py
    module for com with arduino

am_settings.py
    module for program settings

am_tx/am_tx.ino
    arduino code

COPYRIGHT
=========
