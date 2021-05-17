Release 2021-05-10 of the CANPico firmware
==========================================

This is built on the v1.15 release of MicroPython for the Pico with the CANPico board (the firmware
also runs on the CANHack board but the CAN API is not supported).

The pre-built firmware (firmware-20210510.uf2) has a tag of v1.15-canpico and reports this when REPL starts:

    MicroPython v1.15-canpico on 2021-05-10; Raspberry Pi Pico with RP2040

The MD5 hash of the firmware binary firmware-20210510.uf2 is:

afc1fb59d4f678590ff90dbb20725808

The source code for the CANPico support is supplied as a patch. To apply and build the firmware
from source do the following:

1. Get the MicroPython source
-----------------------------

$ git clone http://github.com/micropython/micropython.git
$ cd micropython
$ git checkout v1.15
$ git submodule update --init

2. Add MIN
----------

$ cd lib
$ git clone https://github.com/min-protocol/min.git

3. Build mpy-cross
------------------

$ cd ../mpy-cross
$ make

4. Check this has worked by making the stock Pico firmware
----------------------------------------------------------

$ cd ../ports/rp2/
$ make

5. Apply the patch
------------------

$ git apply canis/v1.15.patch

6. Re-build the firmware with CANPico support
---------------------------------------------

$ make clean
$ make

Install the firmware (in build-PICO/firmware.uf2) on the Pico board in the normal way.
