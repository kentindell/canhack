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
