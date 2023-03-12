## Building firmware for the CANPico

This directory will contain the instructions for building the CANPico firmware against a
MicroPython release (it also will build the firmware for the CHV DEFCON 30 badge), including
source code for the MicroPython API that wraps the CAN C API implementation.

At present there is no official release of MicroPython that supports the Pico W
with wireless features, and building the firmware is a moving target (the firmware
binaries were build against the head of the main branch, which changes on a
daily basis). When there is an official MicroPython release for the Pico W,
the build files will be included here.
