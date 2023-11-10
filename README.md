# imucapture

imucapture is an application for recording and processing data from inertial
measurement units (IMUs).

Full documentation is in the
[user manual](https://github.com/davebuckingham/fish/blob/master/manual/manual.pdf).

imucapture consists of two parts.
First, microcontroller code running on an Arduino board collects data from the IMUs and transmits it to a PC.
Second, a Python program running on the PC receives data from the Arduino and provides
users with a graphical interface. This interface allows users to interact with
the Arduino and to view and manipulate IMU data.

DSF (Dynamic Snap Free) =
assume 5th derivitive of position is 0,
i.e. accelerations are piecewise linear.



