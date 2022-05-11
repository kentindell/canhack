Release 2022-05-11 of the CANPico firmware
==========================================
This is the same as the 2022-04-26 firmware except it has the Python module mincan baked in (the MicroPython firmware build process allows
Python modules to be pre-parsed and included into the firmware without needing the source code on the internal file system). The mincan
module implements a monitor process for communication with a host over MIN and provides certain functions to the host:

- Uploading the serial number of the Pico to the host
- Upload to the host the CAN frames received, CAN frame transmit events and CAN error events
- Sending on CAN frames produced by the host
- Setting the trigger pin TG to fire when certain CAN frames or errors are seen

The monitor is used by the new CANPicoCtrl graphical tool to drive the CANPico interactively from a host PC. The monitor is run as follows:

    from mincan import *
    CANMonitor().run()
    
The pre-built firmware (firmware-20220426.uf2) reports this when REPL starts:

    MicroPython v1.18-194-ge3eae236b on 2022-05-11; Raspberry Pi Pico with RP2040


Release 2022-04-26 of the CANPico firmware
==========================================

This is built on the v1.18 release of MicroPython for the Pico with the CANPico board (the firmware
also runs on the CANHack board but the CAN API is not supported).

The pre-built firmware (firmware-20220426.uf2) reports this when REPL starts:

    MicroPython v1.18-192-g7eb1d835a on 2022-04-26; Raspberry Pi Pico with RP2040

The source code for the CANPico support is supplied as source to drop over the v1.18 release. To build the firmware:

1. Get the MicroPython source
-----------------------------

$ git clone http://github.com/micropython/micropython.git
$ cd micropython
$ git checkout v1.18
$ git submodule update --init

(It's possible to save time by only updating the library modules pico-sdk, tinyusb and axtls: the others aren't needed for the RP2040 firmware build)

2. Add MIN
----------

$ cd lib
$ git submodule add https://github.com/min-protocol/min.git

3. Build mpy-cross
------------------

$ cd ../mpy-cross
$ make

4. Check this has worked by making the stock Pico firmware
----------------------------------------------------------

$ cd ../ports/rp2/
$ make

5. Apply the new code
---------------------

Copy the subfolder 'canis' into ports/rp2

Copy the following six files into ports/rp2 over the top of the v1.18 files:

    CMakeLists.txt
    machine_pin.c
    modrp2.c
    mpconfigport.h
    tusb_config.h
    tusb_port.c

6. Re-build the firmware with CANPico support
---------------------------------------------

$ make clean
$ make

Install the firmware (in build-PICO/firmware.uf2) on the Pico board in the normal way.

NB: there is an issue with the Arm gcc compiler version when building on a Mac. See here for details:

https://github.com/kentindell/canhack/issues/15#issuecomment-1117542310
