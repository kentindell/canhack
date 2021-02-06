## Welcome to the CANHack repository

This repository contains tools and resources for CAN hacking.

- Python tool for creating and parsing CAN bitstreams (``src/canframe.py``)
- The CANHack toolkit (``src/canhack.c``) and the MicroPython bindings and firmware for the Pi Pico (in ``src/rp2``)
- A [Sigrok](https://sigrok.org) protocol decoder for CAN 2.0 (``src/can2`)

There is a [CANHack toolkit demo video](https://youtu.be/dATyoWOlEJU) 
that goes into detail on how to use the toolkit, the CAN protocol hacks it
includes, and demonstrates it attacking CAN frames in real hardware.

There is also a blog post describing the [CANHack toolkit](https://kentindell.github.io/2020/01/20/canhack-toolkit/).

There is a [blog post](https://kentindell.github.io/2021/01/02/can2-wireshark/b) describing the protocol decoder, including how to send frames from a logic analyzer to Wireshark.
