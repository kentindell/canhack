## Welcome to the CANHack repository

This repository contains tools and resources for CAN hacking.

- Python tool for creating and parsing CAN bitstreams (``src/canframe.py``)
- The CANHack toolkit (``src/canhack.c``) and the MicroPython bindings and firmware for the Pi Pico (in ``src/rp2``)
- A [Sigrok](https://sigrok.org) protocol decoder for CAN 2.0 (``src/can2`)

More details on CANHack on [Ken Tindell's](https://kentindell.github.io) web site [here](https://kentindell.github.io/canhack). There
is a [CANHack toolkit demo video](https://youtu.be/dATyoWOlEJU) 
that goes into detail on how to use the toolkit, the CAN protocol hacks it
includes, and demonstrates it attacking CAN frames in real hardware.

More details on the PulseView ``can2`` decoder on [Ken Tindell's](https://kentindell.github.io) web
site [here](https://kentindell.github.io/can2). There
is also a [PulseView and can2 demo video](https://youtu.be/RvExJSDvhKo) showing how to use PulseView
as a logic analyzer and seeing CAN frames at a low-level.
