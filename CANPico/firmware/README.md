## Latest CANPico MicroPython firmware

The firmware for the Pico and Pico W for the CANPico hardware (rev1 and rev2) is in this
directory. Applying the firmware is done in the normal way:

- Press and hold the BOOTSEL button then plug in the Pico or Pico W
- Drag and drop the UF2 firmware file to the drive that mounts

The firmware creates two serial ports, the first is for logging in to the Python REPL
prompt, and the second is intended for communicating with the firmware (for example,
using the MIN protocol to talk to a monitor to send events and commands to a host).

The preferred tool for logging into REPL is the Python tool `rshell` (which can be
used to copy files to the on-board file system, mounted as `/pyboard` within `rshell`).
