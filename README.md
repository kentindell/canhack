## Welcome to the Yes We CAN repository

This repository contains tools and resources for the <b>Yes We CAN</b> project of [Canis Labs](https://canislabs.com),
including:

- Python tool for creating and parsing a CAN bitstreams (``src/canframe.py``)
- A [Sigrok](https://sigrok.org) protocol decoder for CAN 2.0 (``src/can2``)
- Portable generic C for the CANHack toolkit (in ``src``)
- A MicroPython SDK for the Raspberry Pi Pico with the CANHack or CANPico boards,
  released as binary firmware plus also a patch against the v1.18 MicroPython release (in``pico/micropython``)
- Documentation for the CANPico board and the MicroPython SDK (in ``docs``)
- Schematics and Kicad PCB files for the CANHack and CANPico boards (in ``pico/pcb``)
- The CANPicoCtrl graphical UI to control the CANPico board (e.g. turning a CANPico into a logic analyzer trigger board)

There is a [PulseView and can2 demo video](https://youtu.be/RvExJSDvhKo) showing how to use PulseView
as a logic analyzer and seeing CAN frames at a low-level.

The Canis Labs [CTO blog](https://kentindell.github.io) has more information [on the CANPico](https://kentindell.github.io/canpico)
and [on the CANHack toolkit](https://kentindell.github.io/categories#CANHack) - including details on how to make a CANHack board
using breadboard. There is also a [CANHack toolkit demo video](https://youtu.be/dATyoWOlEJU) 
that goes into detail on how to use the toolkit from Python, the CAN protocol hacks it
includes, and demonstrates it attacking CAN frames in real hardware (the video uses the STM32-based PyBoard but
the current MicroPython SDK now runs on the Raspberry Pi Pico).
