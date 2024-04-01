Release 2024-04-01 of the CANPico firmware
==========================================

This is built on the v1.22.2 release of MicroPython for the Pico, 

- The Canis Labs CANPico board (containing a Microchip MCP2517FD CAN controller)
- The Canis Labs CANHack board (containing just a CAN transceiver)
- The Car Hacking Village DEF CON 30 board (containing just a CAN transceiver)

Changes to last release of firmware:
    - Update to v1.22.2 of the upstream MicroPython (which adds Bluetooth support)

There are now multiple builds of the firmware for multiple boards:

fw_20240401_w.uf2           - Firmware for the CANPico + Pico W (the Rev2 boards are supplied with a socketed Pico W)
fw_20240401_p.uf2           - Firmware for the CANPico + Pico

fw_20240401_chcc_w.uf2      - For the CANPico + Pico W plus CANHack and CryptoCAN/HSM APIs
fw_20240401_chcc_p.uf2      - For the CANPico + Pico plus CANHack and CryptoCAN/HSM APIs
fw_20240401_dc30_badge.uf2  - For the DEFCON30 CHV badge
fw_20240401_canhack_uf2     - For the CANHack board

To build the firmware:

1. Get and build the upstream
-----------------------------
Get and build the v1.22.2 firwmare to check that everything work on the upstream.
This will test that the compiler toolchain works.
   
2. Add MIN support and CAN drivers
----------------------------------
$ cd lib
$ git clone https://github.com/min-protocol/min.git
$ git clone https://github.com/kentindell/canis-can-sdk.git

The CANPico firmware provides two USB serial ports on a single USB cable. The first one
is the regular REPL commandline, and the second is used by MIN for communicating with
the host.

3. Patch the firmware
---------------------
Copy the files in this directory into the MicroPython file hierarchy as follows:

File			        Copy to
----                            -------
tusb_config.h                   shared/tinyusb
mp_usbd_descriptor.c            shared/tinyusb
RPI_PICO/mpconfigport.h         ports/rp2/RPI_PICO/mpconfigport.h
RPI_PICO_W/mpconfigport.h       ports/rp2/RPI_PICO_W/mpconfigport.h
canis/*                         ports/rp2/canis/
CMakeLists.txt                  ports/rp2
main.c                          ports/rp2
modrp2.c                        ports/rp2

4. Build the firmware
---------------------
There are several variants of the firmware build. To build specific modules, create
a file with the name of the module (e.g. use the 'touch' command) in the
directory ports/rp2.

The file names are:

CHV_DEFCON30_BADGE (for the DEFCON 30 Car Hacking Village badge)
CANPICO_BOARD (for the Canis Labs CANPico board)

There are other optional modules:

MIN_PROTOCOL (to include the MIN protocol on the second USB serial port)
CANHACK (to include the CANHack class for bit-banging the CAN transceiver)

Then to build for the Pico:

$ make BOARD=RPI_PICO clean
$ make BOARD=RPI_PICO submodules
$ make BOARD=RPI_PICO
$ cp build-RPI_PICO/firmware.uf2 fw_p.uf2

Or to build for the Pico W:

$ make BOARD=RPI_PICO_W clean
$ make BOARD=RPI_PICO_W submodules
$ make BOARD=RPI_PICO_W
$ cp build-RPI_PICO_W/firmware.uf2 fw_w.uf2

Copy the firmware to the Pico hardware in the normal way (i.e. hold down the boot button when
powering on, and copy into the RPI folder that is mounted).

Release 2022-08-01 of the CANPico firmware
==========================================

This is build on the v1.18 release of MicroPython for the Pico with three boards supported:

- The Canis Labs CANPico board (containing a Microchip MCP2517FD CAN controller)
- The Canis Labs CANHack board (containing just a CAN transceiver)
- The Car Hacking Village DEF CON 30 board (containing just a CAN transceiver)

Changes to last release of firmware:
    - Added new class CANOverflow
    - Added receive ISR callback to Python function
    - Added "no overflow" option to prevent overflow events being stored
    - Moved to new Canis CAN SDK for CAN drivers

To build the firmware:

0. Get the tools
----------------

Use the latest gcc cross compiler supplied by Arm. See the blog post:

https://kentindell.github.io/2022/07/26/canpico-c-debug/

for a description of getting the C cross compiler from Arm.

1. Get the MicroPython source
-----------------------------

$ git clone http://github.com/micropython/micropython.git
$ cd micropython
$ git checkout v1.18
$ git submodule update --init

2. Add MIN
----------

$ cd lib
$ git clone https://github.com/min-protocol/min.git
$ cd ..

3. Build mpy-cross
------------------

$ cd mpy-cross
$ make
$ cd ..

4. Check this has worked by making the stock Pico firmware
----------------------------------------------------------

$ cd ports/rp2/
$ cmake CMakeLists.txt
$ make

6. Patch the firmware
---------------------

Copy the file hierarchy in this directory to ports/rp2 in the MicroPython build:

CMakeLists.txt 
machine_pin.c
modrp2.c
tusb_config.h
tusb_port.c
main.c
mpconfigport.h
canis/
    canhack.h
    canhack.c
    common.h
    common.c
    rp2_min.h
    rp2_min.c
    rp2_can.h
    rp2_can.c
    rp2_canhack.h
    rp2_canhack.c

7. Add the Canis CAN SDK
------------------------
$ cd canis
$ git clone https://github.com/kentindell/canis-can-sdk candrivers
$ cd ..

7. Re-build the firmware
---------------------
To do a proper 'clean' between builds, use:

$ rm -rf Makefile CMakeFiles CMakeCache.txt genhdr generated frozen_content.c pico-sdk pioasm frozen_content.c

To make the firmware for the CANPico and CANHack boards:

$ cmake CMakeLists -DCAN=1 -DCANPICO=1
$ make

For the CHV DEF CON 30 badge:

$ cmake CMakeLists -DCAN=1 -DCHV_DEFCON30_BADGE=1
$ make

The firmware is produced in firmware.uf2. Program the Pico with this firmware
in the normal way:

- Press and hold the boot button while powering on the Pico
- Wait until the board mounts as a removable disk
- Copy the firmware into the disk
- Power off the board and power it on again

The board should appear as a USB serial device with two serial ports. On Linux machines, these
will be /dev/ttyACM0 and /dev/ttyACM1, with REPL running on /dev/ttyACM0 (the second serial port
is for MIN to allow automation from a host).

Check the board is running by connecting to the serial port. On Linux the rshell tool is useful
(https://github.com/dhylands/rshell).

Also note that building the TinyUSB drivers will generate a warning:

    warning: 'memset' offset [0, 4095] is out of the bounds [0, 0] [-Warray-bounds]

This can be ignored.

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
    
The pre-built firmware (firmware-20220511.uf2) reports this when REPL starts:

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
