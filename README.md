## Welcome to the Yes We CAN repository

This repository contains tools and resources for the <b>Yes We CAN</b> project of [Canis Labs](https://canislabs.com). There are several projects included in this repository:

- CANHack toolkit (a proof-of-concept CAN protocol hacking library)
- CANPico support (an add-on hardware board for the Raspberry Pi Pico with CAN support)
- CAN SDK for C
- Sigrok CAN protocol decoder (allowing PulseView to decode CAN frames and indicate exceptional protocol events)
- A Python tool for creating CAN frame bit sequences

Canis Labs CTO Dr. Ken Tindell writes about CAN on his blog at: https://kentindell.github.io

## CAN SDK for C (with drivers for the MCP25xxFD)

The CANPico hardware uses the Microchip MCP25xxFD CAN controller (the first revision used the MCP2518FD and the MCP2517FD, the second revision uses the MCP251863 combined controller/transceiver). Drivers for this were included directly in the MicroPython firmware for the CANPico, but have now been carved out into a separate repository as a stand-alone SDK for C. These are available from here: https://github.com/kentindell/canis-can-sdk

## CANHack toolkit

The CANHack toolkit is a proof-of-concept toolkit of different CAN protocol attacks, showing
the viability of low-level bit-banging attacks on the CAN protocol itself.

It is provided as generic C source code in two files:

    src/
        canhack.c
        canhack.h

It has been built into MicroPython firmware for the RP2040 microcontroller on the Raspberry Pi Pico
and will run on the following boards:

- Canis Labs CANHack board
- Canis Labs [CANPico board](https://canislabs.com/canpico/)
- Car Hacking Village DEF CON 30 badge

The firmware binary for MicroPython for the CANHack and CANPico boards are located in:

    pico/
        micropython/
            firmware-20220805.uf2

The firmware binary for MicroPython for the CHV DEF CON 30 badge is located in:

    pico/
        micropython/
            firmware-20220805-CHV-DEFCON30.uf2

Documentation for the MicroPython CANHack API is in:

    docs/
        CANHack MicroPython SDK reference manual.pdf

The Canis Labs [CTO blog](https://kentindell.github.io) has more information
[on the CANHack toolkit](https://kentindell.github.io/categories#CANHack), including details on how 
to make a CANHack board using breadboard. There is also 
a [CANHack toolkit demo video](https://youtu.be/dATyoWOlEJU) that goes into detail on how to
use the toolkit from Python, the CAN protocol hacks it
includes, and demonstrates it attacking CAN frames in real hardware (the video uses the STM32-based PyBoard but the current MicroPython SDK now runs on the Raspberry Pi Pico).

## Python CAN tool

Python tool for creating and parsing a CAN bitstreams (including creating Janus attack frames):

    src/
        canframe.py

## Sigrok CAN protocol decoder

A [Sigrok](https://sigrok.org) protocol decoder for CAN 2.0:

    src/
        can2/
            __init__.py
            pd.py

There is a [PulseView and can2 demo video](https://youtu.be/RvExJSDvhKo) showing how to use PulseView
as a logic analyzer and seeing CAN frames at a low-level.

The [CIA CAN newsletter](https://https://can-newsletter.org) has published an [article](https://can-newsletter.org/tools/tools-miscellaneous/210607_can-decoder-warns-for-malicious-attacks_cnlm_ken-tindell) describing the protocol decoder and showing how it can spot some CAN protocol attacks.

## Canis Labs CANPico support

A MicroPython SDK for the Raspberry Pi Pico with a CAN API is provided as firmware:

    pico/
        micropython/
            firmware-20220805.uf2

See the instructions in:

    pico/
        micropython/
            README.txt

for building the firmware (the firmware uses the Canis CAN SDK)

The documentation for the CANPico is in:

    docs/
        CANPico MicroPython SDK reference manual.pdf
        CANPico hardware reference manual.pdf

Schematics for the CANHack and CANPico boards are in:

    pico/
        pcb/
            CANHack_schm.pdf
            CANPico_pA4_schm.pdf

Kicad source files for the CANHack and CANPico boards are in:

    pico/
        pcb/
            CANHack.kicad_pcb
            CANPico.kicad_pcb

