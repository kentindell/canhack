## Welcome to the Yes We CAN repository

This repository contains tools and resources for the <b>Yes We CAN</b> project of [Canis Labs](https://canislabs.com). There are several related projects here:

- Support for the [Canis Labs](https://canislabs.com) [CANPico](https://canislabs.com/canpico) (an add-on CAN hardware board for the Raspberry Pi Pico)
- CANHack toolkit (a proof-of-concept low-level CAN protocol hacking library)
- Sigrok CAN protocol decoder (allowing PulseView to decode CAN frames and indicate exceptional low-level protocol events)
- A Python tool for creating CAN frame bit sequences

Canis Labs CTO Dr. Ken Tindell writes about CAN on his blog at: https://kentindell.github.io

## Canis Labs CANPico hardware support

MicroPython firmware and documentation is in the `CANPico` folder. In addition, there is support for
C included in the Canis Labs CAN SDK for C, which has drivers for the MCP25xxFD (the MicroPython firmware
uses this CAN SDK to provide a MicroPython CAN API).

The Canis Labs CAN SDK repository is:

[https://github.com/kentindell/canis-can-sdk](https://github.com/kentindell/canis-can-sdk)

and contains a "hello world" application using the CAN API, with pre-built firmware for the Pico and Pico W
with the CANPico board.

## CANHack toolkit

The CANHack toolkit is a proof-of-concept toolkit of different CAN protocol attacks, showing
the viability of low-level bit-banging attacks on the CAN protocol itself.

It is provided as generic C source code in two files:

    src/
        canhack.c
        canhack.h

It has been built into the Canis Labs MicroPython firmware for the Raspberry Pi Pico and Pico W
for the following hardware:

- Canis Labs [CANPico board](https://canislabs.com/canpico/)
- Canis Labs CANHack board (this uses the same firmware as the CANPico)
- Car Hacking Village DEF CON 30 badge

The MicroPython firmware for the CHV DEF CON 30 badge is located in:

    pico/
        micropython/
            firmware-20220805-CHV-DEFCON30.uf2

Documentation for the MicroPython CANHack API is in:

    CANPico/
        docs/
            CANHack MicroPython SDK reference manual.pdf

The Canis Labs [CTO blog](https://kentindell.github.io) has more information
[on the CANHack toolkit](https://kentindell.github.io/categories#CANHack), including details on how 
to make a CANHack board using breadboard. There is also 
a [CANHack toolkit demo video](https://youtu.be/dATyoWOlEJU) that goes into detail on how to
use the toolkit from Python, the CAN protocol hacks it
includes, and demonstrates it attacking CAN frames in real hardware (NB: the video uses the STM32-based PyBoard,
but the API is the same).

## Sigrok CAN protocol decoder

A [Sigrok](https://sigrok.org) protocol decoder for CAN 2.0:

    src/
        can2/
            __init__.py
            pd.py

There is a [PulseView and can2 demo video](https://youtu.be/RvExJSDvhKo) showing how to use PulseView
as a logic analyzer and seeing CAN frames at a low-level.

The [CIA CAN newsletter](https://https://can-newsletter.org) has published an [article](https://can-newsletter.org/tools/tools-miscellaneous/210607_can-decoder-warns-for-malicious-attacks_cnlm_ken-tindell) describing the protocol decoder and showing how it can spot some CAN protocol attacks.

## Python CAN tool

Python tool for creating and parsing a CAN bitstreams (including creating Janus attack frames):

    src/
        canframe.py

