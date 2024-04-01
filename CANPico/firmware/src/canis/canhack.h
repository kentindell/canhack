// The CANHack toolkit
// ===================
//
// Created by ken (https://kentindell.github.io) on 11/12/2019. This code is licensed under the MIT license:
//
// -------------------------------------------------------------------------------------------------------------------
// Copyright 2020 Dr. Ken Tindell (https://kentindell.github.io)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
// to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of
// the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// -------------------------------------------------------------------------------------------------------------------
//
// The CANHack toolkit is a bit-banging implementation of part of the CAN protocol to enable attacks on CAN.
//
// The toolkit is portable and requires access to GPIO pins for CAN TX and CAN RX and access to a timer.
//
// Porting
// =======
//
// CANHACK_BOARD_H should be defined with the filename of the header file that contains the definitions. An
// example header file is given for the Raspberry Pi Pico that uses an RP2040 microcontroller, called canhack-rp2.h.
//
// There are inline functions that must be defined so that canhack.c can deal with the machine-dependent driving of
// the I/O pins.
//
// The toolkit currently spins in software to detect the key events (start/end of bit, sample point, falling edge on
// CAN TX). Some microcontrollers will have timer/counter hardware that can support this more directly (such as
// putting a value on an output pin when the timer matches a value). But this toolkit is designed for maximum
// portability and to show how hijacking a microcontroller with a CAN transceiver connected to an arbitrary I/O
// port could hack the CAN protocol.
//
// Using the API
// =============
//
// 1.  The first step is to initialize CANHack with the canhack_init() call. This takes a bit time (in CPU cycles, the same
//     timebase in the CPU clock macros). It also takes a sample point as clock cycles from the start of the bit. The
//     time between events must be long enough that the software has run before the next event occurs, so the sample point
//     cannot be too close to the end of bit. 75% of a bit time should be OK.
//
// 2.  The second step is to define the properties of the CAN frame in the hack. A handle to the frame is obtained
//     by the canhack_get_frame() call (two frames can be defined, and the call takes a parameter to indicate whether
//     the first or second frame is being set). The canhack_set_frame() call is used to set the parameters of the frame.
//     The CAN ID is split into 11 bit and 18 bit extension parts, since this is how the CAN protocol actually works.
//     The MicroPython wrapper for the toolkit calculates these values from a single integer depending on whether the
//     frame is marked standard or extended.
//
// 3a. If the CAN frame is to be transmitted on the bus then the canhack_send_frame() or canhack_send_janus_frame()
//     call can be made. This enters the frame into arbitration and transmits it. A timeout and a retries parameter
//     can be set (if retries is 0 then the call will return if it loses arbitration or there is an error during
//     transmission after arbitration).
//
// 3b. Alternatively, if the CAN frame is to be used as a template for a protocol attack then the mask/match values used
//     to target a frame should be set. This is done by calling canhack_set_attack_masks(). Then an attack call can be made.
//
// 4a. To mount a Bus-off attack, a Double Receive Attack, or a Freeze Doom Loop Attack, use the canhack_error_attack()
//     call.
// 4b. To mount a spoof attack, use the canhack_spoof_frame() call.
//
// 4c. To mount an error passive spoof attack, uise the canhack_spoof_frame_error_passive() call.
//
// The specifics of each API call are documented below.

#ifndef CANHACK_H
#include <inttypes.h>
#include <stdbool.h>

#ifndef CANHACK_BOARD_H
#define CANHACK_BOARD_H     "rp2_canhack.h"
#endif

#include CANHACK_BOARD_H
#include <stdio.h>

#define CANHACK_MAX_BITS                        (160U)

/// Structure that defines a CAN frame parameters
typedef struct {
    uint8_t tx_bitstream[CANHACK_MAX_BITS];     ///< The bitstream of the CAN frame
    bool stuff_bit[CANHACK_MAX_BITS];           ///< Indicates if the corresponding bit is a stuff bit
    uint8_t tx_bits;                            ///< Number of  bits in the frame
    uint32_t tx_arbitration_bits;               ///< Number of bits in arbitartion (including stuff bits); the fields are ID A + RTR (standard) or ID A + SRR + IDE + ID B + RTR (extended)

    // Fields set when creating the CAN frame
    uint32_t crc_rg;                            ///< CRC value (15 bit value)
    uint32_t last_arbitration_bit;              ///< Bit index of last arbitration bit (always the RTR bit for both IDE = 0 and IDE = 1); may be a stuff bit
    uint32_t last_dlc_bit;                      ///< Bit index of last bit of DLC field; may be a stuff bit
    uint32_t last_data_bit;                     ///< Bit index of the last bit of the data field; may be a stuff bit
    uint32_t last_crc_bit;                      ///< Bit index of last bit of the CRC field; may be a stuff bit
    uint32_t last_eof_bit;                      ///< Bit index of the last bit of the EOF field; may be a stuff bit
    bool frame_set;                             ///< True when the frame has been set; may be a stuff bit

    // Fields used during creation of the CAN frame
    uint32_t dominant_bits;                     ///< Dominant bits in a row
    uint32_t recessive_bits;                    ///< Recessive bits in a row
    bool stuffing;                              ///< True if stuffing enabled
    bool crcing;                                ///< True if CRCing enabled
} canhack_frame_t;

/// \brief Initialize CANHack toolkit
void canhack_init(void);

/// \brief Set the parameters of the CAN frame
/// \param id_a 11-bit CAN ID
/// \param id_b 18-bit extension to CAN ID (used if ide=true)
/// \param rtr true if frame is remote
/// \param ide true if frame is extended
/// \param dlc DLC field; data length if a data frame, arbitrary 4-bit value if a remote frame
/// \param data pointer to up to 8 bytes of payload
/// \param frame the handle to the frame (see canhack_get_frame)
void canhack_set_frame(uint32_t id_a, uint32_t id_b, bool rtr, bool ide, uint32_t dlc, const uint8_t *data, canhack_frame_t *frame);

/// \brief Get handle to frame
/// \param second true if frame 2 is wanted
/// \return handle to frame
canhack_frame_t *canhack_get_frame(bool second);

/// \brief Set the attack masks from frame 1 (frame 1 must be set)
void canhack_set_attack_masks(void);

/// \brief Send a square wave on the CAN TX pin (used to check setup)
void canhack_send_square_wave(void);

/// \brief Send to the CAN TX pin what is seen on the CAN RX pin (used for testing)
void canhack_loopback(void);

/// \brief Send a CAN frame to the CAN bus without waiting for 11 idle bits or syncing with SOF
void canhack_send_raw_frame(void);

/// \brief Send a frame on the CAN bus (not an attack)
/// \param retries Number of times the frame should be re-entered into arbitration (after losing or after an error)
/// \param second Whether to use the second CAN frame buffer
/// \return True if frame was sent OK, false if timed out or too many retries
bool canhack_send_frame(uint32_t retries, bool second);

/// \brief Send a Janus frame on the CAN bus
/// \param sync_time Time of dominant state at start of bit
/// \param split_time Time when phase 1 bit value is set to phase 2 bit value
/// \param retries Number of times the frame should be re-entered into arbitration (after losing or after an error)
/// \return
bool canhack_send_janus_frame(ctr_t sync_time, ctr_t split_time, uint32_t retries);

/// \brief Send a spoofed frame just after the target frame ends
/// \param janus True if the spoof is a Janus frame
/// \param sync_time Time of dominant state at start of bit (if a Janus frame)
/// \param split_time Time when phase 1 bit value is set to phase 2 bit value (if a Janus frame)
/// \param retries Number of times the frame should be re-entered into arbitration (after losing or after an error)
/// \return True if frame was sent OK, false if timed out or too many retries
bool canhack_spoof_frame(bool janus, ctr_t sync_time, ctr_t split_time, uint32_t retries);

/// \brief Overwrite the target frame (sender must be in error passive mode)
/// \param loopback_offset Time to shift the bit pattern to align with the spoofed frame
/// \return False if the timeout occurred or there was an error, true if the spoof frame was sent
bool canhack_spoof_frame_error_passive(uint32_t loopback_offset);

/// \brief Destroy a frame and generate errors
/// \param repeat Number of times to repeat the attack at the end of the targeted frame
/// \param inject_error True if an error should be injected after the end of arbitration when the frame is seen
/// \param eof_mask The mask for which bits at the end of the targeted frame should be used to trigger the attack
/// \param eof_match The values corresponding to each bit in the mask
/// \return True if the attack succeeded and false if the timeout occurred
bool canhack_error_attack(uint32_t repeat, bool inject_error, uint32_t eof_mask, uint32_t eof_match);

/// \brief Set the timeout for an operation
/// \param timeout Timeout, in units of polling loops (target-specific real-time value)
void canhack_set_timeout(uint32_t timeout);

/// \brief Stop the current operation running
void canhack_stop(void);

#define CANHACK_H

#endif //CANHACK_H
