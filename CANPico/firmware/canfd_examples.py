# CAN FD CANHack — example / bring-up snippets
#
# Paste these at the MicroPython REPL of a Pico running the CANHACK_FD firmware
# (a plain Pico + CAN transceiver, CANHack-board wiring: CAN_RX = GP21 / pin 27,
# CAN_TX = GP22 / pin 29).  Fixed bit rates: 500 kbit/s arbitration, 2 Mbit/s data.
#
# See CANFD_STATUS.md for the full test plan.

import machine
import rp2

# 0) Confirm the FD overclock is active (MUST print 250000000)
print("sys freq:", machine.freq())

# 1) Create the toolkit. Argument is the ARBITRATION bit rate in kbit/s.
#    500 -> 2 Mbit/s data, 250 -> 1 Mbit/s data, 125 -> 500 kbit/s data.
ch = rp2.CANHack(bit_rate=500)

# Identify the running firmware (uname-like): git hash + build timestamp.
print(ch.build_info())        # e.g. 'canhack_fd g1a2b3c4 built 2026-10-21 14:59:12 UTC'

# 2) Build a CAN FD frame (standard ID, BRS on -> fast data phase, ESI error-active).
ch.set_frame(0x123, data=bytes([0x01, 0x02, 0x03, 0x04,
                                0x05, 0x06, 0x07, 0x08]),
             fd=True, brs=True, esi=True)
ch.print_frame()          # inspect the constructed bitstream + stuff bits (no bus needed)


def scope_loop():
    # Continuously transmit so the Siglent can trigger on it.
    # Expect: 2.0 us arbitration bits, 0.5 us data bits after BRS.
    while True:
        ch.send_frame()


def send_once():
    # Single send; with a PCAN-USB FD on the bus (500k/2M) this frame should be
    # received with no CRC/form error.
    ch.send_frame()


def sweep_payloads():
    # Exercise CRC-17 (<=16 bytes) and CRC-21 (>16 bytes) and both BRS settings.
    for dlc_bytes in (8, 16, 20, 64):
        data = bytes((i & 0xff) for i in range(dlc_bytes))
        for brs in (True, False):
            ch.set_frame(0x123, data=data, fd=True, brs=brs, esi=True)
            ch.send_frame()


def janus_fd():
    # CAN FD Janus attack (known to be marginal in the fast phase; classical CAN is reliable).
    ch.set_frame(0x123, data=bytes([0x01]), fd=True, brs=True)                 # main
    ch.set_frame(0x123, data=bytes([0x02]), fd=True, brs=True, second=True)    # shadow
    # Both frames must have identical total bit length and identical BRS position/value.
    ch.send_janus_frame(sync_time_fd=31, split_time_fd=78)


# Cross-check used in the test plan: if brs=False is accepted but brs=True is not,
# the problem is specifically the 2 Mbit/s data-phase timing, not the CRC maths.
def send_no_brs():
    ch.set_frame(0x123, data=bytes([0x01, 0x02, 0x03, 0x04,
                                    0x05, 0x06, 0x07, 0x08]),
                 fd=True, brs=False, esi=True)
    ch.send_frame()
