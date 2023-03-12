## CANPico support

This folder contains support files for the Canis Labs CANPico. There are two revisions of the CANPico
but the firmware operates on either version. There is also support for the Raspberry Pi Pico
and the Raspberry Pi Pico W (a Pico with wireless support). The CANPico firmware for MicroPython
includes the CANHack Toolkit as well as a CAN API.

### Hardware

There are two revisions of the hardware:

- Rev 1 uses the MCP2518FD (or the MCP2517FD) chip with an MCP2562FD CAN transceiver chip
- Rev 2 uses the MCP251863 combined CAN controller/transceiver chip

The MCP251863 is functionally an MCP2518FD CAN controller, and the Rev 2 board behaves as the
Rev 1 board and uses the same firmware.

The pinouts of both revisions are the same (although rev2 comes with solder bridges that
can be cut to free up three GPIO pins to make it easier to add other Pico boards).

![CANPico pinout](./CANPico%20pinout.png)

Coupled to the CANPico board is a Raspberry Pi Pico or a Raspberry Pi Pico W. There is
different firmware for each: the Pico W firmware includes support for its on-board WiFi
and the new location of its on-board user LED.

The folder `pcb` contains the board design files (the CANPico was designed with Kicad).

### C SDK

The CANPico is supported by the Canis Labs C SDK with the MCP25xxFD driver. This SDK
is in the following GitHub repository and can be used by anyone with MCP25xxFD
CAN controller hardware:

[https://github.com/kentindell/canis-can-sdk](https://github.com/kentindell/canis-can-sdk)

The CAN drivers have recently been updated to implement workarounds to silicon errata in
the MCP25xxFD.

There is a "hello world" example in the repository that will build for the Pico and the
Pico W with a CANPico board.

### MicroPython SDK

The CANPico is supported with custom MicroPython firmware that provides an API for
CAN. The latest firmware is the 20230309 release and is found in the following files:

- `firmware/firmware-20230309-beta.uf2`
- `firmware/firmware-20230309-picow-beta.uf2`

This firmware is marked 'beta' because it is forked from an intermediate release of the
MicroPython firmware: the last official release of MicroPython pre-dates the Pico W
and so more recent codebase has been used. There are changes coming in MicroPython
that will be rolled into the CANPico firmware before the 'beta' status will be lifted
(most notably new official support for multiple virtual USB serial ports).

The source code for this firmware will be released at the next official MicroPython
release: merging the files for the new classes into the upstream firmware is
delicate surgery and hitting a moving target makes that extra specially difficult.

Older CANPico firmware can be found in the subfolder `firmware/OLD`. Note that older
firmware doesn't explicitly support the Pico W and although it will run on the Pico W,
none of the wireless functionality is supported.

Changes introduced by the 20230309 firmware:

- Now using the latest Canis Labs C SDK drivers for the MCP25xxFD
- Fixes to the API (e.g. the recv() calls now return an empty tuple rather than an empty list)
- Default `tx_open_drain` parameter in the `CAN()` constructor is now `True`
- New error message if the CAN controller will not set the TX open drain mode
- `get_diagnostics()` now has 6 counters, including a count of the times Bus Off happened
- Can now explicitly pass `None` to various calls to select the default

### Programming the MicroPython firmware

The firmware is programmed into a Pico or Pico W in the normal way. If programming fails
then because the bootloader is in ROM and can't be affected, it's likely the `.uf2` file has
become corrupted.

### Documentation

The documentation is available in the `docs` folder, and previous releases are in `docs/OLD`.
There is currently documentation for:

- CANPico hardware (which includes schematics for both revisions of the CANPico)
- CANPico MicroPython SDK reference manual
- CANHack MicroPython reference manual

A cheatsheet for the MicroPython API is also included.

The CAN SDK for C reference manual for the CANPico is located in the Canis Labs CAN SDK
repository given above, and examples code for the CANPico with Pico and Pico W are also 
there.

### CryptoCAN encryption of CAN frames

A MicroPython API for the Canis Labs emulated SHE HSM and CryptoCAN scheme is included
in a special release of the firmware that can be downloaded directly from the
[CryptoCAN section](https://canislabs.com/cryptocan) of the Canis Labs web site. There is
also documentation, a quick start guide, and videos.

An [article on encryption on CAN bus](https://can-newsletter.org/magazine/51-December%202022/) 
with CryptoCAN has been published in the [CAN In Automation newsletter](https://can-newsletter.org).
CryptoCAN was also presented at
the [2022 ASRG Secure our Streets conference](https://sos.asrg.io/schedule-and-presentations/).

### Other hardware

A CANHack board can be made with just breadboard and a Pico (or Pico W) to explore the
concept of CAN protocol hacking. The CANPico firmware here support such a board (but
obviously the `CAN` class will not talk to a CAN controller). There is
a [blog post on how to make a CANHack board](https://kentindell.github.io/2021/02/06/canhack-pico/)
at [Ken Tindell's blog site](https://kentindell.github.io).
