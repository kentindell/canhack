CANPicoCtrl v1.0
================

This is the first version of the graphical tool CANPicoCtrl to interactively control the CANPico.

CANPico monitor
---------------
The CANPico must be running the monitor program and connected via USB. The monitor is written in Python
and run with the following:

    from mincan import *
    CANMonitor().run()

This can be executed from REPL or put into main.py.

The Python code for the monitor is baked into the latest CANPico firmware.

CANPicoCtrl.exe
---------------
This is an executable file for Windows. It does not require installation (it caches files in
a temporary directory for performance). The CANPico monitor must be running for this to communicate
with the CANPico.
