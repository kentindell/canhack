Release 2021-07-13 of the CANPico firmware
==========================================

This is built on the v1.16 release of MicroPython for the Pico with the CANPico board (the firmware
also runs on the CANHack board but the CAN API is not supported).

The pre-built firmware (firmware-20210713.uf2) has a tag of v1.16-canpico and reports this when REPL starts:

    MicroPython v1.16-canpico on 2021-07-13; Raspberry Pi Pico with RP2040

The MD5 hash of the firmware binary firmware-20210713.uf2 is:

    823db2bc6fef1082ef0cd8d3cf81f4c0

The source code for the CANPico support is supplied as a patch to v1.15. To apply and build the firmware
from source do the following:

0. Start from the v1.15 patch baseline
--------------------------------------

This is in OLD/v1.15.patch

1. Get the MicroPython source
-----------------------------

$ git clone http://github.com/micropython/micropython.git
$ cd micropython
$ git checkout v1.16
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

$ git apply canis/v1.16.patch

6. Re-build the firmware with CANPico support
---------------------------------------------

$ make clean
$ make

Install the firmware (in build-PICO/firmware.uf2) on the Pico board in the normal way.
