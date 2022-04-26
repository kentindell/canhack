"""

Example and utility functions for CANPico board
-----------------------------------------------

This file can be installed on the Pico's file system using the Thonny IDE.

Example functions are included in the CANPico class and are imported
and used like this from a REPL prompt:

>>> from canpico import *
>>> c = CAN()
>>> cp = CANPico(c)
>>> cp.identify()

"""

from struct import pack, unpack
from machine import Pin
from time import sleep_ms
from rp2 import CAN, CANFrame, CANID, CANHack, MIN
from utime import sleep, sleep_ms

urgent = CANFrame(CANID(0x10), data=b'urgent')
slow = CANFrame(CANID(0x700))
medium_frames = [CANFrame(CANID(0x300 + i), data=bytes([i])) for i in range(10)]
sync_frames = [CANFrame(CANID(0)), CANFrame(CANID(1), data=b'deadbeef')]


class CANPico:
    def __init__(self, c, ch=None):
        self.c = c
        self.ch = ch  # CANHack instance (CAN controller should be in tx_open_drain mode)

    # Flash the Pico board LED to identify which board it is
    def identify(self, period_ms=1000):
        led = Pin(25, Pin.OUT)
        while True:
            led.high()
            sleep_ms(period_ms // 2)
            led.low()
            sleep_ms(period_ms // 2)

    # Ping function that sends back a CAN frame
    def pinger(self):
        while True:
            frames = self.c.recv()
            for frame in frames:
                self.c.send_frame(frame)

    # Simple CAN bus monitor
    def mon(self):
        while True:
            frames = self.c.recv()
            for frame in frames:
                print(frame)

    # Sends a frame repeatedly, if period_ms is None sends back-to-back
    def sender(self, f, period_ms=100):
        while True:
            if period_ms is not None:
                sleep_ms(period_ms)
            try:
                self.c.send_frame(f)
            except:
                pass

    # Send a periodic CAN frame with an incrementing payload
    def periodic(self, can_id=CANID(0x123), period_ms=100, i=0):
        while True:
            f = CANFrame(can_id, data=pack('>I', i))
            if period_ms is not None:
                sleep_ms(period_ms)
            try:
                self.c.send_frame(f)
                i += 1
            except:
                pass

    # Send a frame and block until it is sent
    def send_wait(self, f):
        if not isinstance(f, CANFrame):
            raise TypeError("f is not a CAN frame")
        self.c.send_frame(f)
        while True:
            if f.get_timestamp() is not None:
                return

    # Sends a frame and if queueing failed then keep trying until there is space
    def always_send(self, f):
        while True:
            try:
                self.c.send_frame(f)
                return
            except:
                pass

    # Wait until there is space and then send a frame
    def always_send2(self, f):
        while True:
            if self.c.get_send_space() > 0:
                self.c.send_frame(f)
                return

    # Create a follow-up message with the timestamp in the payload
    def fup(self, f):
        if not isinstance(f, CANFrame):
            raise TypeError("f is not a CAN frame")
        return CANFrame(CANID(0x101), data=pack('>I', f.get_timestamp()))

    # Sends a heartbeat frame every 1s and a follow-up message containing the local transmit timestamp
    def heartbeat(self):
        f = CANFrame(CANID(0x100))
        while True:
            self.send_wait(f)
            f2 = self.fup(f)
            self.c.send_frame(f2)
            sleep(1)

    # Measure the clock drift between two CANPico boards
    def drift(self):
        self.c.recv()  # Clear out old frames
        ts = None  # Don't know the first timestamp yet
        while True:
            frames = self.c.recv()
            for frame in frames:
                if frame is not None:
                    if frame.get_arbitration_id() == 0x100:  # First frame
                        ts = frame.get_timestamp()
                    if frame.get_arbitration_id() == 0x101:  # Follow-up frame
                        sender_ts = unpack('>I', frame.get_data())[0]
                        if ts is not None:  # If the first timestamp is known then the offset can be computed
                            print(sender_ts - ts)

    # Send a spoof frame via CANHack
    def spoof(self, f, timeout=500000):
        self.ch.set_frame(can_id=f.get_arbitration_id(), extended=f.is_extended(), remote=f.is_remote(), data=f.get_data())
        self.ch.spoof_frame(timeout=timeout)

    # Simple CAN bus uploading monitor
    def minmon(self):
        m = MIN()
        while True:
            b = self.c.recv(as_bytes=True)
            if b:
                m.send_frame(1, b)
            m.recv()


    # Wait for a frame
    def recv_wait(self):
        self.c.recv()  # Throw away any left-over pending frames
        while True:
            frames = self.c.recv()
            if len(frames) > 0:
                return frames

    # Send a frame after receiving a frame
    def sync_send_frame(self, f):
        self.recv_wait()
        self.c.send_frame(f)

    # Send frames after receiving a frame
    def sync_send_frames(self, frames, fifo=False):
        self.recv_wait()
        self.c.send_frames(frames, fifo=fifo)

