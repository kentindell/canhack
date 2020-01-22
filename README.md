## Welcome to the CANHack repository

This repository contains tools and resources for CAN hacking.

The ``src`` folder contains ``canframe.py`` that provides the ``CANFrame`` class that generates the bitstream of a CAN frame.

Also included are ``canhack.h`` and ``canhack.c``, the generic C for the
``CANHack`` toolkit. The subfolder `pyb` contains files that port the toolkit
to the PyBoard and make the API available in MicroPython. To patch the
MicroPython firmware, also ports/stm32/Makefile and ports/stm32/modpyb.c need
editing to build the files and include the `CANHack` class.

There is a [CANHack toolkit demo video](https://youtu.be/dATyoWOlEJU) 
that goes into detail on how to use the toolkit, the CAN protocol hacks it
includes, and demonstrates it attacking CAN frames in real hardware.

There is also a blog post describing the [CANHack toolkit](https://kentindell.github.io/2020/01/20/canhack-toolkit/).
