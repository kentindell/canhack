// Copyright 2020 Canis Automotive Labs (https://canislabs.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdbool.h>

#include "common.h"
#include "rp2_can.h"

#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <py/objstr.h>
#include <py/stream.h>
#include <py/runtime.h>
#include <py/mphal.h>
#include <py/mperrno.h>
#include <py/runtime.h>

#include <hardware/structs/scb.h>

// TODO faster FIFO implementation using power-of-two masks on index values
// TODO add option to reject all remote frames (where RX handler decides whether to reject before putting into software FIFO)
// TODO speed up ISRs: calculate buffer addr by shadowing TEF and RX FIFO rather than use an SPI transaction to pick it up
// TODO add low-power standby mode to API (put controller into standby, put transceiver into standby via XSTBY pin)
// TODO more than TRIG pin 1 trigger with an OR condition between them
// TODO create option discard the overflow 'None' in the list of received frames returned by recv()

// Debug options: CAN IRQ checking looks to see if interrupts are locked out over critical sections
// #define CAN_IRQ_CHECKING
// #define CAN_DEBUG

#ifdef CAN_DEBUG
#define CAN_DEBUG_PRINT(fmt, args...)       printf(fmt, ##args)
#else
#define CAN_DEBUG_PRINT(fmt, args...)       /* */
#endif

#ifdef CAN_IRQ_CHECKING
STATIC bool irq_locked = false;
#define CAN_ASSERT(cond, msg)               {if (!(cond)) {mp_printf(MP_PYTHON_PRINTER, (msg));}}
#else
#define CAN_ASSERT(cond, msg)               /* */
#endif

// Simple GPIO functions should be inlined to ensure they are in RAM when time critical (can't rely on the
// compiler to inline them into RAM)
#define SPI_SELECT()                        (sio_hw->gpio_clr = (1U << SPI_CS_GPIO))
#define SPI_DESELECT()                      (sio_hw->gpio_set = (1U << SPI_CS_GPIO))

#define CRITICAL_SECTION_CHECK(s)           CAN_ASSERT(irq_locked, s)

#define TRIG_SET()                          (sio_hw->gpio_set = (1U << TRIG_GPIO))
#define TRIG_CLEAR()                        (sio_hw->gpio_clr = (1U << TRIG_GPIO))
#define NOP()                               __asm__("nop");

#define XSTBY_SET()                         (sio_hw->gpio_set = (1U << XSTBY_GPIO))
#define XSTBY_CLEAR()                       (sio_hw->gpio_clr = (1U << XSTBY_GPIO))

#ifdef CAN_IRQ_CHECKING
#define DISABLE_GPIO_INTERRUPTS()           (irq_set_enabled(IO_IRQ_BANK0, false), irq_locked = true)
#define ENABLE_GPIO_INTERRUPTS()            (irq_set_enabled(IO_IRQ_BANK0, true), irq_locked = false)
#else
#define DISABLE_GPIO_INTERRUPTS()           (irq_set_enabled(IO_IRQ_BANK0, false))
#define ENABLE_GPIO_INTERRUPTS()            (irq_set_enabled(IO_IRQ_BANK0, true))
#endif

#define TRIG_GPIO                           (2U)
#define XSTBY_GPIO                          (3U)
#define SPI_CS_GPIO                         (6U)
#define LEVEL_SENSITIVE_LOW                 (1U)
#define EDGE_SENSITIVE_RISING               (1U << 3)
#define MCP251718FD_SPI                     (spi1)
#define SPI_GPIO_IRQ_PRIORITY               (1U << 6) // Default IRQ priority is 0x80 (i.e. 2, where 0 is the highest and 3 is the lowest).

STATIC bool mcp251718fd_init(canmode_t mode, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw);

void can_init(void) {
    // Set up the root pointer to a null CAN controller object so that the memory is not allocate until CAN is used.
    MP_STATE_PORT(rp2_can_obj) = NULL;
}

void can_deinit(void) {
    // Called when the system is soft reset (CTRL-D in REPL).

    // If the controller is initialized then take it offline and deactivate it
    if (MP_STATE_PORT(rp2_can_obj) != NULL) {
        // Lock out interrupts while we are altering the system

        DISABLE_GPIO_INTERRUPTS();
        // Don't want any CAN interrupts hanging over for receiving, transmitting,
        // etc. if the root pointer has been de-allocated.
        gpio_set_irq_enabled(SPI_IRQ_GPIO, LEVEL_SENSITIVE_LOW, false);

        // The pins must have been set by the constructor so leave them as they were set until the next
        // time the constructor runs. The TX open drain status will remain (so if the controller
        // was offline then it will stay offline and stay in open drain)

        // Ask the controller to go offline so that it won't continue to babble afterwards
        // Note that this will take some time if there is an ongoing frame transmission/reception,
        // but eventually it will go offline.
        mcp251718fd_init(CAN_MODE_OFFLINE, 4U, 10U, 3U, 2U);

        // Also no need now for the root pointer to exist and we can garbage collect this
        MP_STATE_PORT(rp2_can_obj) = NULL;

        // Now clear to re-enable all GPIO interrupts
        ENABLE_GPIO_INTERRUPTS();
        // Depending on IRQ latching in the CPU it's possible that there may be spurious interrupts
        // but these have guards against null pointers
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Start of MCP2517/18FD SPI drivers /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Set up the pins on the Pico to interface to the MCP2517/18FD
STATIC void pico_pin_init(void)
{
    // These are the defaults anyway:
    //    spi_set_format(spi1, 8U, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // CANPico board is clocked at 40MHz, so SPI must be no more than 18500000 (according to data sheet)
    spi_init(MCP251718FD_SPI, 18500000);
    gpio_set_function(8, GPIO_FUNC_SPI);        // SPI1_Rx
    gpio_set_function(10, GPIO_FUNC_SPI);       // SPI1_SCK
    gpio_set_function(11, GPIO_FUNC_SPI);       // SPI1_Tx

    // Set XSTBY pin to software controlled
    gpio_set_function(XSTBY_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(XSTBY_GPIO, GPIO_OUT);
    // Set the XSTBY pin to 0 to enable the transceiver
    XSTBY_CLEAR();

    // Set TRIGGER pin to software controlled
    gpio_set_function(TRIG_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(TRIG_GPIO, GPIO_OUT);
    // Set the TRIG pin to 0
    TRIG_CLEAR();

#ifdef CANPICO_DEBUG_PIN
    // Set debug pin to software controlled
    gpio_set_function(DEBUG_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(DEBUG_GPIO, GPIO_OUT);
    // Set the DEBUG pin to 0
    DEBUG_CLEAR();
#endif

    gpio_set_function(SPI_CS_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(SPI_CS_GPIO, GPIO_OUT);
    // Deselect the MCP2517/18FD
    gpio_set_mask(1U << SPI_CS_GPIO);
}

// RP2040 is little-endian, SPI reads the words in little endian format (first byte is lowest bits)
STATIC TIME_CRITICAL void mcp251718fd_spi_write_word(uint32_t addr, uint32_t word)
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C1");

    uint8_t buf[6];
    // MCP2517/18FD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x20 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;
    buf[2] = word & 0xffU;
    buf[3] = (word >> 8) & 0xffU;
    buf[4] = (word >> 16) & 0xffU;
    buf[5] = (word >> 24) & 0xffU;

    // SPI transaction
    // The Pico is little-endian so the first byte sent is the lowest-address, which is the
    // same as the RP2040
    SPI_SELECT();
    spi_write_blocking(MCP251718FD_SPI, buf, sizeof(buf));
    SPI_DESELECT();
}

STATIC TIME_CRITICAL void mcp251718fd_spi_write_4words(uint16_t addr, const uint32_t words[])
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C2");

    // Prepare a contiguous buffer for the command because the SPI hardware is pipelined and do not want to stop
    // to switch buffers
    uint8_t cmd[18];
    // MCP2517/18FD SPI transaction = command/addr, 4 bytes
    cmd[0] = 0x20 | ((addr >> 8U) & 0xfU);
    cmd[1] = addr & 0xffU;

    uint32_t i = 2U;
    for (uint32_t j = 0; j < 4U; j++) {
        cmd[i++] = words[j] & 0xffU;
        cmd[i++] = (words[j] >> 8) & 0xffU;
        cmd[i++] = (words[j] >> 16) & 0xffU;
        cmd[i++] = (words[j] >> 24) & 0xffU;
    }

    // SPI transaction
    SPI_SELECT();
    spi_write_blocking(MCP251718FD_SPI, cmd, sizeof(cmd));
    SPI_DESELECT();
}

STATIC TIME_CRITICAL uint32_t mcp251718fd_spi_read_word(uint16_t addr)
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C3");

    uint8_t cmd[6];
    uint8_t resp[6];

    cmd[0] = 0x30 | ((addr >> 8U) & 0xfU);
    cmd[1] = addr & 0xffU;
    // TODO can remove the following because not strictly necessary (but useful for debugging with a logic analyzer)
    cmd[2] = 0xdeU;
    cmd[3] = 0xadU;
    cmd[4] = 0xbeU;
    cmd[5] = 0xefU;

    // SPI transaction
    SPI_SELECT();
    spi_write_read_blocking(MCP251718FD_SPI, cmd, resp, sizeof(cmd));
    SPI_DESELECT();

    uint32_t word = ((uint32_t)resp[2]) | ((uint32_t)resp[3] << 8) | ((uint32_t)resp[4] << 16) | ((uint32_t)resp[5] << 24);
    return word;
}

STATIC TIME_CRITICAL void mcp251718fd_spi_read_words(uint16_t addr, uint32_t *words, uint32_t n)
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C4");

    uint8_t buf[2];

    // MCP2517/18FD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x30 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;

    // SPI transaction
    SPI_SELECT();
    // Send command, which flushes the pipeline then resumes
    spi_write_blocking(spi1, buf, 2U);
    // Bulk data
    spi_read_blocking(spi1, 0xaa, (uint8_t *)(words), 4U * n);
    SPI_DESELECT();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// End of MCP2517/18FD SPI drivers /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Start of MCP2517/18FD drivers //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
STATIC TIME_CRITICAL void init_tx_buffers(rp2_can_obj_t *self);

#define         OSC             (0xe00U)
#define         IOCON           (0xe04U)
#define         CRC             (0xe08U)
#define         ECCCON          (0xe0cU)
#define         ECCSTAT         (0xe10U)
#define         DEVID           (0xe14U)
#define         C1CON           (0x000U)
#define         C1NBTCFG        (0x004U)
#define         C2DBTCFG        (0x008U)
#define         C1TDC           (0x00cU)
#define         C1TBC           (0x010U)
#define         C1TSCON         (0x014U)
#define         C1VEC           (0x018U)
#define         C1INT           (0x01cU)
#define         C1RXIF          (0x020U)
#define         C1TIF           (0x024U)
#define         C1RXOVIF        (0x028U)
#define         C1TXATIF        (0x02cU)
#define         C1TXREQ         (0x030U)
#define         C1TREC          (0x034U)
#define         C1BDIAG0        (0x038U)
#define         C1BDIAG1        (0x03cU)
#define         C1TEFCON        (0x040U)
#define         C1TEFSTA        (0x044U)
#define         C1TEFUA         (0x048U)
#define         C1TXQCON        (0x050U)
#define         C1TXQSTA        (0x054U)
#define         C1TXQUA         (0x058U)
#define         C1FIFOCON1      (0x05cU)
#define         C1FIFOSTA1      (0x060U)
#define         C1FIFOUA1       (0x064U)
#define         C1FLTCON(n)     (((n) * 4U) + 0x1d0U)
#define         C1FLTOBJ(n)     (((n) * 8U) + 0x1f0U)
#define         C1MASK(n)       (((n) * 8U) + 0x1f4U)

// Hard reset of the MCP2517/18FD
STATIC void mcp251718fd_reset(void)
{
    uint8_t buf[2] = {0, 0};

    SPI_SELECT();
    spi_write_blocking(MCP251718FD_SPI, buf, 2U);
    SPI_DESELECT();
}

STATIC TIME_CRITICAL void mcp251718fd_set_pins(bool tx_open_drain)
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C5");

    // Set SYSCLK to 40MHz, the external crystal, and don't use the PLL
    mcp251718fd_spi_write_word(OSC, 0);
    // Set up IOCON by setting:
    // SOF=1 to select SOF on CLKO
    // TXCANOD=1 to select open collector transmit pin
    // PM0=1 to use pin as GPIO1
    // PM0=1 to use pin as GPIO0
    // TRIS1=1 to select GPIO1 as an input
    // TRIS0=1 to select GPIO0 as an input
    uint32_t word = (1U << 29) | (1U << 25) | (1U << 24) | (1U << 1) | (1U << 0);
    if (tx_open_drain) {
        word = 1U << 28;
    }
    mcp251718fd_spi_write_word(IOCON, word);
}

// This is called after the mode change to normal has occurred so that there won't be a interrupt
// coming from configuration to normal mode (which would be confused with a bus-off interrupt)
STATIC TIME_CRITICAL void mcp251718fd_enable_interrupts(void)
{
    CRITICAL_SECTION_CHECK("C6");

    // Enable interrupts
    // IVMIE -  CAN error
    // CERRIE - CAN error status (error passive, bus-off, etc.)
    // TEFIE - transmit event FIFO
    // RXIE - receive FIFO
    // Dismiss all interrupt flags
    mcp251718fd_spi_write_word(C1INT, (1U << 31) | (1U << 29) | (1U << 20) |(1U << 17));
}

// Request config mode, returns true if request accepted and configured
// After a hard reset this should always succeed
STATIC bool mcp251718fd_init(canmode_t mode, uint32_t brp, uint32_t tseg1, uint32_t tseg2, uint32_t sjw)
{
    // NB: The MCP2517/18FD pins must have been initialized before calling this function

    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C7");

    // Request config mode
    mcp251718fd_spi_write_word(C1CON, 4U << 24);

    bool config_mode = ((mcp251718fd_spi_read_word(C1CON) >> 21) & 0x7U) == 4U;

    if (config_mode) {
        // Set clock and I/O pins (set TX pin to open drain if offline because GPIO pin connected to TX might be used)
        mcp251718fd_set_pins(false);
        // Set bit rate according to profile
        // FSYSCLK is 40MHz, 25ns clock period

        mcp251718fd_spi_write_word(C1NBTCFG, (brp << 24) | (tseg1 << 16) | (tseg2 << 8) | sjw);

        // Set timestamping counter
        // Set prescaler to /40 to count microseconds
        mcp251718fd_spi_write_word(C1TSCON, (1U << 16) | 39U);

        // Transmit event FIFO control register
        // FSIZE 32-deep
        // TEFTSEN Timestamp transmissions
        // TEFNEIIE not empty interrupt enable
        mcp251718fd_spi_write_word(C1TEFCON, (0x1fU << 24) | (1U << 5) | (1U << 0));

        // Transmit queue control register
        // FSIZE 32-deep
        // TXAT Unlimited retransmissions (this field isn't active but set it anyway)
        mcp251718fd_spi_write_word(C1TXQCON, (0x1fU << 24) | (3U << 21));

        // FIFO 1 is the receive FIFO
        // FSIZE 32-deep
        // RXTSEN Timestamp receptions
        // TFNRFNIE interrupts enabled
        mcp251718fd_spi_write_word(C1FIFOCON1, (0x1fU<< 24) | (1U << 5) | (1U << 0));

        mcp251718fd_enable_interrupts();
        ///// THIS COMES LAST: WILL SET EVERYTHING RUNNING

        // Enable transmit queue, store in transmit event FIFO, CAN 2.0 mode
        // Select mode
        uint32_t reqop;
        switch (mode) {
            default:
            case CAN_MODE_NORMAL:
                reqop = 6U;
                break;
            case CAN_MODE_LISTEN_ONLY:
                reqop = 3U;
                break;
            case CAN_MODE_ACK_ONLY:
                reqop = 7U;
                break;
            case CAN_MODE_OFFLINE:
                reqop = 4U;
                break;
        }
        mcp251718fd_spi_write_word(C1CON, (1U << 19) | (1U << 20) | (reqop << 24));
    }

    return config_mode;
}

// Return the device ID
STATIC uint32_t mcp251718fd_hard_reset(void)
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C8");

    // Reset the controller, set the clocks
    mcp251718fd_reset();
    mcp251718fd_set_pins(false);
    return mcp251718fd_spi_read_word(DEVID);
}

// This is the main function for transmitting a frame
// Returns false if no room
STATIC TIME_CRITICAL bool mcp251718fd_send_frame(rp2_can_obj_t *self, rp2_canframe_obj_t *frame, bool fifo, can_tx_queue_t *tx_queue, can_tx_fifo_t *tx_fifo)
{
    // Must be called with interrupts locked
    CRITICAL_SECTION_CHECK("C9");

    // (it can be called from ISR or from background)

    if (!fifo || tx_queue->fifo_slot == MCP251718FD_TX_QUEUE_SIZE) {
        // Put the frame in the transmit queue
        if (tx_queue->num_free_slots == 0) {
            // No room in the transmit queue
            CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "No room in the transmit queue\n");
            return false;
        }
        else {
            // Write frame to a transmit queue message slot
            // This must fit into 16 bits because the buffer space in total is only 2Kbytes, and starts from 0x400
            uint32_t c1txqsta = mcp251718fd_spi_read_word(C1TXQSTA);
            if ((c1txqsta & (1U << 0)) == 0) {
                // Queue full, can't write to it, should not have happened because now inconsistent with
                // software counters
                self->txqsta_bad++;
                return false;
            }

            uint16_t c1txqua = (uint16_t)mcp251718fd_spi_read_word(C1TXQUA);
            uint16_t addr = c1txqua + 0x400U;
            // (Transmit event slots start at an offset of 0 (a total of 3 x 4 bytes x 32 slots = 384 bytes)
            // Transmit queue slots start at an offset 0x180, and each is 16 bytes (we allocated 16 bytes to the
            // payload even though handling only CAN frames, meaning the whole buffer slot is 16 bytes)
            uint32_t free_slot = (c1txqua - 0x180U) >> 4;

            if (free_slot >= MCP251718FD_TX_QUEUE_SIZE) {
                self->txqua_bad++;
                return false;
            }

            CAN_ASSERT(free_slot < MCP251718FD_TX_QUEUE_SIZE, "Z0");

            // Copy the frame into the message slot in the controller
            // Layout of TXQ message object:
            uint32_t t[4];
            t[0] = ((frame->can_id & 0x3ffffU) << 11) | ((frame->can_id >> 18) & 0x7ff);
            t[1] = (free_slot << 9) | frame->dlc;
            CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "seq=%d\n",tx_queue->free_slot);
            if (frame->can_id & (1U << 29)) {
                t[1] |= (1U << 4);    // IDE
            }
            if (frame->remote) {
                t[1] |= (1U << 5);
            }
            // Payload words are in little endian format: first byte is bits 0-7
            t[2] = frame->payload[0];
            t[3] = frame->payload[1];

            // Mark slot and update next free slot
            if (fifo) {
                tx_queue->fifo_slot = free_slot;
            }
            tx_queue->num_free_slots--;
            tx_queue->frames[free_slot] = frame;

            frame->timestamp_valid = false;
            CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "new free_slot=%d\n",tx_queue->free_slot);

            // TODO could use a DMA channel and chain these SPI transactions using DMA
            // Write this block over SPI
            CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "t[0]=0x%08"PRIx32"\n", t[0]);
            mcp251718fd_spi_write_4words(addr, t);

            // Now tell the controller to take the frame and move C1TXQUA
            // Set UINC=1, TXREQ=1
            // Transmit queue control register
            mcp251718fd_spi_write_word(C1TXQCON, (1U << 8) | (1U << 9));

            return true;
        }
    }
    else {
        if (tx_fifo->num_free_slots == 0) {
            CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "No room in the FIFO\n");
            // No room in the FIFO
            return false;
        }
        else {
            // Head of FIFO is in the priority queue so put this frame into the TX FIFO
            tx_fifo->num_free_slots--;
            tx_fifo->frames[tx_fifo->tail_idx++] = frame;
            if(tx_fifo->tail_idx == MCP251718FD_TX_FIFO_SIZE) {
                // Wrap the tail index
                tx_fifo->tail_idx = 0;
            }
            frame->timestamp_valid = false;
            return true;
        }
    }
}

// At present there is a single CAN controller on the board so there is no need to work out which
// device is interrupting, etc.

// Called to deal with a frame that has been transmitted (actually the TEF IRQ on the device)
STATIC TIME_CRITICAL void mcp251718fd_tx_handler(void)
{
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj);
    if (self == NULL) {
        // Spurious interrupt: a race with the interrupt being raised and the source of interrupts being
        // disabled
        return;
    }

    // Find out which frame was sent (using the SEQ field), get the frame object handle,
    // fill in the timestamp, remove the frame from the software queue, adding a FIFO queue
    // frame if necessary
    uint16_t addr = (uint16_t)mcp251718fd_spi_read_word(C1TEFUA) + 0x400U;

    // Don't care about the CAN frame ID (we know it already), just SEQ and timestamp
    uint32_t details[2];
    mcp251718fd_spi_read_words(addr + 4U, details, 2U);
    uint8_t seq = (uint8_t)(details[0] >> 9);

    // The sequence number may have been corrupted over SPI by noise so we treat it with some
    // suspicion to avoid a buffer overflow.
    if (seq > MCP251718FD_TX_QUEUE_SIZE) {
        self->seq_bad++;
    }
    seq &= (0x1fU);

    CAN_ASSERT(seq <= MCP251718FD_TX_QUEUE_SIZE, "A1");

    uint32_t timestamp = details[1];

    bool fifo = (seq == self->tx_queue.fifo_slot);

    // Remove frame from the transmit queue
    if (fifo) {
        self->tx_queue.fifo_slot = MCP251718FD_TX_FIFO_SIZE;
    }
    // Update frame's timestamp and fetch its tag

    rp2_canframe_obj_t *seq_frame = self->tx_queue.frames[seq];
    // Frame should be defined but because we don't quite trust seq being
    // true we add an extra guard here.
    if (seq_frame != NULL) {
        seq_frame->timestamp = timestamp;
        seq_frame->timestamp_valid = true;
        uint32_t tag = seq_frame->tag;
        // Remove frame from transmit queue (most of the management of the free space in the
        // transmit queue is done by the hardware, but we keep track of how many free slots
        // to save SPI transactions asking for them)
        self->tx_queue.num_free_slots++;

        // Remove reference to frame so it can eventually be garbage collected if appropriate
        self->tx_queue.frames[seq] = NULL;

        // If this is a FIFO frame and there are more FIFO frames, then queue that one
        if (fifo && self->tx_fifo.num_free_slots < MCP251718FD_TX_FIFO_SIZE) {
            self->tx_fifo.num_free_slots++;
            rp2_canframe_obj_t *fifo_frame = self->tx_fifo.frames[self->tx_fifo.head_idx];
            // Zero out the frame so that there is no reference to it for later garbage collection
            self->tx_fifo.frames[self->tx_fifo.head_idx] = NULL;
            // Pop head of transmit FIFO
            self->tx_fifo.head_idx++;
            if (self->tx_fifo.head_idx == MCP251718FD_TX_FIFO_SIZE) {
                // Wrap the head index
                self->tx_fifo.head_idx = 0;
            }
            mcp251718fd_send_frame(self, fifo_frame, true, &self->tx_queue, &self->tx_fifo);
        }

        ////// Keep track of the transmit event //////
        // TODO the FIFO code should really be made generic (can't use compiler to inline due to XIP issue)
        if (self->event_fifo.free <= 1U) {
            // No space for it - discard event, marking the last free slot as an "overflow"; this will become
            // a None object when get_tx_events() is called
            if (self->event_fifo.free == 0) {
                // Do nothing: there must already an overflow event at the back of the FIFO
                self->event_fifo.events[self->event_fifo.dropped_event_idx].info.overflow_cnt++;
            } else {
                // Add an 'overflow' event to the back of the queue
                self->event_fifo.free = 0;
                uint8_t idx = self->event_fifo.tail_idx++;
                self->event_fifo.dropped_event_idx = idx;
                if (self->event_fifo.tail_idx == TX_EVENT_FIFO_SIZE) {
                    self->event_fifo.tail_idx = 0;
                }
                self->event_fifo.events[idx].event_type = EVENT_TYPE_OVERFLOW;
                // The tag is used as a dropped event counter
                self->event_fifo.events[idx].info.overflow_cnt = 0;
                // The timestamp is the time of the first drop
                self->event_fifo.events[idx].timestamp = timestamp;
            }
        } else {
            // Put the event into the FIFO
            self->event_fifo.free--;
            uint8_t idx = self->event_fifo.tail_idx++;
            if (self->event_fifo.tail_idx == TX_EVENT_FIFO_SIZE) {
                self->event_fifo.tail_idx = 0;
            }
            self->event_fifo.events[idx].timestamp = timestamp;
            self->event_fifo.events[idx].info.frame_tag = tag;
            self->event_fifo.events[idx].event_type = EVENT_TYPE_TRANSMITTED_FRAME;
        }
    }
    else {
        //
        self->seq_bad++;
    }

    // Pop event in controller, keep timestamps enabled, keep not-empty interrupts enabled
    // MCP2517/18FD interrupts are level-sensitive so GPIO must be set to level sensitive; interrupt
    // will be re-raised if still not empty when serviced.
    // Set FSIZE, UINC, TEFTSEN, TEFNEIIE
    mcp251718fd_spi_write_word(C1TEFCON, (0x1fU << 24) | (1U << 8) | (1U << 5) | (1U << 0));
}

STATIC TIME_CRITICAL void mcp251718fd_bus_off_handler(void)
{
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj);
    if (self == NULL) {
        // Spurious interrupt: a race with the interrupt being raised and the source of interrupts being
        // disabled
        return;
    }
    // Frames should not be accepted for transmission if not yet in normal mode.
    uint32_t c1trec = mcp251718fd_spi_read_word(C1TREC);

    if (c1trec & (1U << 21)) { // TXBO
        // Bus-off will erase the transmit queues, so all the frames we queued are now going to be
        // thrown away, so discard them in the software queues too

        // Discard all the frames
        init_tx_buffers(self);
    }
    // This could be warning about error passive and other states, but we don't care about those
    // Dismisses CERRIF interrupts (and others)
    mcp251718fd_enable_interrupts();
}

STATIC TIME_CRITICAL void mcp251718fd_error_handler(void)
{
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj);
    if (self == NULL) {
        // Spurious interrupt: a race with the interrupt being raised and the source of interrupts being
        // disabled
        return;
    }

    // Read the time (won't be very accurate because of the time taken to get here)
    uint32_t timestamp = mcp251718fd_spi_read_word(C1TBC);

    // Trigger for error
    if (self->triggers[0].enabled && self->triggers[0].on_error) {
        TRIG_SET();
    }

    if (self->recv_errors) {
        // Get information about the error
        uint32_t c1bdiag1 = mcp251718fd_spi_read_word(C1BDIAG1);

        // Put the error in to the receive FIFO since it's kind of a received thing, even if not a frame
        if (self->rx_fifo.free <= 1U) {
            // No space for it - mark the last slot as an overflow and then future error / received frames can
            // increment the counter
            if (self->rx_fifo.free == 0) {
                // There must already an overflow frame at the back of the queue
                self->rx_fifo.rx_events[self->rx_fifo.dropped_event_idx].info.overflow_cnt.error_cnt++;
                if (self->rx_fifo.rx_events[self->rx_fifo.dropped_event_idx].info.overflow_cnt.error_cnt == 0) {
                    // Overflowed so roll it back to make it sticky
                    self->rx_fifo.rx_events[self->rx_fifo.dropped_event_idx].info.overflow_cnt.error_cnt--;
                }
            } else {
                // Add an 'overflow' frame to the back of the queue
                self->rx_fifo.free = 0;
                uint8_t idx = self->rx_fifo.tail_idx++;
                self->rx_fifo.dropped_event_idx = idx;
                if (self->rx_fifo.tail_idx == RX_FIFO_SIZE) {
                    self->rx_fifo.tail_idx = 0;
                }
                self->rx_fifo.rx_events[idx].event_type = EVENT_TYPE_OVERFLOW;
                // Initialize the counters
                self->rx_fifo.rx_events[idx].info.overflow_cnt.error_cnt = 1U;  // Did not record this error frame
                self->rx_fifo.rx_events[idx].info.overflow_cnt.frame_cnt = 0;
                // The timestamp is the time of the first drop
                self->rx_fifo.rx_events[idx].timestamp = timestamp;
            }
        } else {
            // Put the error into the FIFO
            self->rx_fifo.free--;
            uint8_t idx = self->rx_fifo.tail_idx++;
            if (self->rx_fifo.tail_idx == RX_FIFO_SIZE) {
                self->rx_fifo.tail_idx = 0;
            }
            self->rx_fifo.rx_events[idx].event_type = EVENT_TYPE_CAN_ERROR;
            self->rx_fifo.rx_events[idx].timestamp = timestamp;
            self->rx_fifo.rx_events[idx].info.c1bdiag1 = c1bdiag1;
            self->rx_fifo.rx_events[idx].info.c1bdiag1 = c1bdiag1;
        };
    }

    mcp251718fd_enable_interrupts();
    // Trigger held high for enough time to be seen by even a slow logic analyzer
    // Will still be able to trigger even if not storing errors in the receive FIFO
    TRIG_CLEAR();
}

// Called with a received frame
// TODO performance enhancement: calculate addr by shadowing RX FIFO rather than use an SPI transaction to pick it up
STATIC TIME_CRITICAL void mcp251718fd_rx_handler()
{
    uint16_t addr = (uint16_t)mcp251718fd_spi_read_word(C1FIFOUA1) + 0x400U;

    // Pick up the frame
    uint32_t r[5];
    mcp251718fd_spi_read_words(addr, r, 5U);
    // Mark the frame as taken, ensure that timestamping and the not-empty interrupt are still enabled
    // Set UINC, RXTSEN, TFNRFNIE,
    mcp251718fd_spi_write_word(C1FIFOCON1, (1U << 8) | (1U << 5) | (1U << 0));

    // Assemble CAN ID from ID A, ID B and IDE
    canid_t canid = ((r[0] >> 11) & 0x3ffffU) | ((r[0] & 0x7ff) << 18) | ((r[1] & (1U << 4)) << 25);
    uint8_t dlc = r[1] & 0xfU;
    bool remote = (r[1] & (1U << 5)) != 0;
    uint8_t id_filter = (r[1] >> 11) & 0x1fU;
    uint32_t timestamp = r[2];
    uint32_t payload_0 = r[3];
    uint32_t payload_1 = r[4];
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj);

    if (self->triggers[0].enabled && self->triggers[0].on_rx) {
        if (((canid & self->triggers[0].can_id_mask) == self->triggers[0].can_id_match) &&
            ((dlc & self->triggers[0].can_dlc_mask) == self->triggers->can_dlc_match) &&
            ((payload_0 & self->triggers[0].can_payload_mask[0]) == self->triggers->can_payload_match[0]) &&
            ((payload_1 & self->triggers[0].can_payload_mask[1]) == self->triggers->can_payload_match[1])) {
            TRIG_SET();
        }
    }

    // The FIFO elements in the software receive FIFO are not MicroPython CANFrame objects but structures with
    // CAN info to allow a MicroPython object to be created.
    if (self->rx_fifo.free <= 1U) {
        // No space for it - discard a frame, marking the last free slot as an "overrun"; this will become
        // a None object when recv() is called
        if (self->rx_fifo.free == 0) {
            // There must already an overflow frame at the back of the queue
            self->rx_fifo.rx_events[self->rx_fifo.dropped_event_idx].info.overflow_cnt.frame_cnt++;
            if (self->rx_fifo.rx_events[self->rx_fifo.dropped_event_idx].info.overflow_cnt.frame_cnt == 0) {
                // Overflowed so roll it back to make it sticky
                self->rx_fifo.rx_events[self->rx_fifo.dropped_event_idx].info.overflow_cnt.frame_cnt--;
            }
        }
        else {
            // Add an 'overflow' frame to the back of the queue
            self->rx_fifo.free = 0;
            uint8_t idx = self->rx_fifo.tail_idx++;
            self->rx_fifo.dropped_event_idx = idx;
            if (self->rx_fifo.tail_idx == RX_FIFO_SIZE) {
                self->rx_fifo.tail_idx = 0;
            }
            self->rx_fifo.rx_events[idx].event_type = EVENT_TYPE_OVERFLOW;
            // Use CAN ID as a dropped-frame count
            self->rx_fifo.rx_events[idx].info.overflow_cnt.frame_cnt = 1U;  // Did not record this frame
            self->rx_fifo.rx_events[idx].info.overflow_cnt.error_cnt = 0;
            // The timestamp is the time of the first drop
            self->rx_fifo.rx_events[idx].timestamp = timestamp;
        }
    }
    else {
        // Put the frame into the FIFO
        self->rx_fifo.free--;
        uint8_t idx = self->rx_fifo.tail_idx++;
        if (self->rx_fifo.tail_idx == RX_FIFO_SIZE) {
            self->rx_fifo.tail_idx = 0;
        }
        self->rx_fifo.rx_events[idx].event_type = EVENT_TYPE_RECEIVED_FRAME;
        self->rx_fifo.rx_events[idx].timestamp = timestamp;
        self->rx_fifo.rx_events[idx].info.rx_frame.canid = canid;
        self->rx_fifo.rx_events[idx].info.rx_frame.dlc = dlc;
        self->rx_fifo.rx_events[idx].info.rx_frame.remote = remote;
        self->rx_fifo.rx_events[idx].info.rx_frame.payload[0] = payload_0;
        self->rx_fifo.rx_events[idx].info.rx_frame.payload[1] = payload_1;
        self->rx_fifo.rx_events[idx].info.rx_frame.id_filter = id_filter;
    };
    // Trigger held high for enough time to be seen by even a slow logic analyzer
    TRIG_CLEAR();
}

// Called directly from outer handler
void TIME_CRITICAL mcp251718fd_irq_handler(void)
{
    // Gets an interrupt from:
    //
    // Sent frame
    // Received frame
    // Bus-off

    // Can call a single sub-handler and each sub-handler will handle one
    // event; if there are multiple events then the IRQ will be re-raised
    // (since it is level-sensitive). Interrupt will also be re-raised if
    // more events have occurred.

#ifdef CAN_IRQ_CHECKING
    irq_locked = true;
#endif
    // Read C1INT and then trigger on:
    uint32_t events = mcp251718fd_spi_read_word(C1INT);
    if (events & (1U << 4)) {           // TEFIF (i.e. TEF event)
        mcp251718fd_tx_handler();
    } else if (events & (1U << 1)) {    // RXIF (i.e. received frame into the FIFO)
        mcp251718fd_rx_handler();
    } else if (events & (1U << 13)) {   // CERRIF to detect bus off
        mcp251718fd_bus_off_handler();
    } else if (events & (1U << 15)) {   // IVMIF to detect errors
        mcp251718fd_error_handler();
    }
    else {
        CAN_ASSERT(0, "A2");
        // Spurious interrupt - may be handled by a MicroPython function in the handoff ISR
    }
#ifdef CAN_IRQ_CHECKING
    irq_locked = false;
#endif
}

STATIC TIME_CRITICAL uint32_t mcp251718fd_get_timebase(void)
{
    CAN_ASSERT(!irq_locked, "A3")
    DISABLE_GPIO_INTERRUPTS();
    uint32_t timebase = mcp251718fd_spi_read_word(C1TBC);
    ENABLE_GPIO_INTERRUPTS();

    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "timebase=0x%08"PRIx32"\n", timebase);

    return timebase;
}

STATIC TIME_CRITICAL uint32_t mcp251718fd_get_trec(void)
{
    CAN_ASSERT(!irq_locked, "A4")
    DISABLE_GPIO_INTERRUPTS();
    uint32_t trec = mcp251718fd_spi_read_word(C1TREC);
    ENABLE_GPIO_INTERRUPTS();

    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "trec=0x%08"PRIx32"\n", trec);

    return trec;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// End of MCP2517/18FD drivers ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Start of MicroPython bindings //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
STATIC TIME_CRITICAL void init_tx_buffers(rp2_can_obj_t *self)
{
    CRITICAL_SECTION_CHECK("CA");

    // Ensure there are no references to any CANFrame instances
    for (uint32_t i = 0; i < MCP251718FD_TX_FIFO_SIZE; i++) {
        self->tx_fifo.frames[i] = 0;
    }
    self->tx_fifo.head_idx = 0;
    self->tx_fifo.tail_idx = 0;
    self->tx_fifo.num_free_slots = MCP251718FD_TX_FIFO_SIZE;

    // Ensure there are no references to any CANFrame instances
    for (uint32_t i = 0; i < MCP251718FD_TX_QUEUE_SIZE; i++) {
        self->tx_queue.frames[i] = 0;
    }
    self->tx_queue.num_free_slots = MCP251718FD_TX_QUEUE_SIZE;
    self->tx_queue.fifo_slot = MCP251718FD_TX_FIFO_SIZE;
}

STATIC TIME_CRITICAL void init_structures(rp2_can_obj_t *self)
{
    CRITICAL_SECTION_CHECK("CB");

    // Receive frame FIFO is empty
    self->rx_fifo.head_idx = 0;
    self->rx_fifo.tail_idx = 0;
    self->rx_fifo.free = RX_FIFO_SIZE;
    self->rx_fifo.dropped_event_idx = 0;

    // Transmit event FIFO is empty
    self->event_fifo.head_idx = 0;
    self->event_fifo.tail_idx = 0;
    self->event_fifo.free = TX_EVENT_FIFO_SIZE;
    self->event_fifo.dropped_event_idx = 0;

    // Initialize transmit buffers
    init_tx_buffers(self);

    // Initialize the event triggers
    self->triggers[0].enabled = false;
    self->triggers[0].on_error = false;
}

////////////////////////////////////// Start of CAN class //////////////////////////////////////
// Create the CAN instance and initialize the controller
STATIC mp_obj_t rp2_can_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_profile,           MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 0}},
        {MP_QSTR_id_filters,        MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_hard_reset,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},
        {MP_QSTR_brp,               MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = -1}},
        {MP_QSTR_tseg1,             MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 10}},
        {MP_QSTR_tseg2,             MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 3}},
        {MP_QSTR_sjw,               MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 2}},
        {MP_QSTR_recv_errors,       MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},
        {MP_QSTR_mode,              MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_tx_open_drain,     MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},
        // TODO add "accept_remote" parameter (default True)
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t profile = args[0].u_int;
    mp_obj_dict_t *id_filters = args[1].u_obj;
    bool hard_reset = args[2].u_bool;
    int brp = args[3].u_int;
    u_int tseg1 = args[4].u_int;
    u_int tseg2 = args[5].u_int;
    u_int sjw = args[6].u_int;
    bool recv_errors = args[7].u_bool;
    canmode_t mode = args[8].u_int;
    bool tx_open_drain = args[9].u_int;

    // Modes are:
    // 0: (default) CAN.NORMAL, start normally
    // 1: CAN.LISTEN_ONLY, does not ever set TX to 0
    // 2: CAN.ACK_ONLY, does not transmit but does set ACK=0
    // 3: CAN.OFFLINE, does not send or receive

    if (id_filters != NULL) {
        // Check dictionary is well-formed
        if(!MP_OBJ_IS_TYPE(id_filters, &mp_type_dict)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "A dict expected for id_filters"));
        }

        for(uint32_t idx = 0; idx < 32U; idx++) {
            mp_map_elem_t *elem = mp_map_lookup(&id_filters->map, MP_OBJ_NEW_SMALL_INT(idx), MP_MAP_LOOKUP);
            if (elem != NULL) {
                if(!MP_OBJ_IS_TYPE(elem->value, &rp2_canidfilter_type)) {
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "A CANIDFilter instance expected for filter %d", idx));
                }
            }
        }
    }

    // Create class instance for controller
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj);

    if (MP_STATE_PORT(rp2_can_obj) == NULL) {
        // Newly create object (we don't want it created always because it's a fairly large object, with
        // large receive FIFO and this shouldn't be allocated until needed).
        self = m_new_obj(rp2_can_obj_t);
        self->base.type = &rp2_can_type;
        MP_STATE_PORT(rp2_can_obj) = self;
    }

    CAN_ASSERT(!irq_locked, "A5")
    DISABLE_GPIO_INTERRUPTS();

    // Sets up SPI1, ensures chip not selected
    pico_pin_init();

    if (hard_reset) {
        mcp251718fd_hard_reset();
    }

    // Set clock and I/O pins (set TX pin to open drain if requested)
    mcp251718fd_set_pins(tx_open_drain);
    // Set bit rate according to profile
    // FSYSCLK is 40MHz, 25ns clock period

    // Try to put the device into config mode (which resets everything)
    uint32_t retries = 0;
    for (;;) {
        // We should give up after some time: could be many milliseconds or even forever if the
        // bus is under attack with the freeze doom loop
        if (brp < 0) {
            switch (profile) {
                default:
                case CAN_BITRATE_500K_75:
                    brp = 4U;       // 40MHz / 5 = 8MHz, 16 time quanta per bit
                    tseg1 = 10U;    // Sync seg is 1
                    tseg2 = 3U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_250K_75: // 250bit/sec, 75%
                    brp = 9U;       // 40MHz / 10 = 8MHz, 16 time quanta per bit
                    tseg1 = 10U;
                    tseg2 = 3U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_125K_75:
                    brp = 19U;      // 40MHz / 20 = 8MHz, 16 time quanta per bit
                    tseg1 = 10U;
                    tseg2 = 3U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_1M_75:
                    brp = 1U;       // 40MHz / 2 = 20MHz, 20 time quanta per bit
                    tseg1 = 13U;
                    tseg2 = 4U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_500K_50:
                    brp = 4U;       // 40MHz / 5 = 8MHz, 16 time quanta per bit
                    tseg1 = 6U;     // Sync seg is 1
                    tseg2 = 7U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_250K_50:
                    brp = 9U;       // 40MHz / 10 = 8MHz, 16 time quanta per bit
                    tseg1 = 6U;     // Sync seg is 1
                    tseg2 = 7U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_125K_50:
                    brp = 19U;      // 40MHz / 20 = 8MHz, 16 time quanta per bit
                    tseg1 = 6U;     // Sync seg is 1
                    tseg2 = 7U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_1M_50:
                    brp = 1U;       // 40MHz / 2 = 20MHz, 20 time quanta per bit
                    tseg1 = 8U;     // Sync seg is 1
                    tseg2 = 9U;
                    sjw = 2U;
                    break;
                case CAN_BITRATE_2M_50:
                    brp = 0;
                    tseg1 = 8U;
                    tseg2 = 9U;
                    sjw = 1U;
                    break;
                case CAN_BITRATE_4M_90:
                    brp = 0;
                    tseg1 = 7U;
                    tseg2 = 0;
                    sjw = 1U;
                    break;
                case CAN_BITRATE_2_5M_75:
                    brp = 1;
                    tseg1 = 4U;
                    tseg2 = 1U;
                    sjw = 1U;
                    break;
                case CAN_BITRATE_2M_80:
                    brp = 0U;
                    tseg1 = 14U;
                    tseg2 = 3U;
                    sjw = 1U;
                    break;
            }
        }
        if (mcp251718fd_init(mode, brp, tseg1, tseg2, sjw)) {
            break;
        }
        if (retries++ > 1000U) {
            ENABLE_GPIO_INTERRUPTS();
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "Cannot put MCP2517/18FD into config mode"));
        }
    }

    uint32_t filter_control[8];
    // Disable all the ID filters (which might not be disabled if we come into this with the controller
    // having already been running)
    for (uint32_t i = 0; i < 8U; i++) {
        filter_control[i] = 0;
        mcp251718fd_spi_write_word(C1FLTCON(i), 0);
    }

    // Add in the filters
    if (id_filters != NULL) {
        CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Setting specific filters\n");
        for (uint32_t idx = 0; idx < 32U; idx++) {
            mp_map_elem_t *elem = mp_map_lookup(&id_filters->map, MP_OBJ_NEW_SMALL_INT(idx), MP_MAP_LOOKUP);
            if (elem != NULL) {
                CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Filter index=%d\n", idx);
                rp2_canidfilter_obj_t *filter = elem->value;

                // Enables the filter and sets it to direct frames to RX FIFO 1
                filter_control[idx >> 2] |= (0x81U << ((idx & 0x03U) << 3));
                // Sets the mask/match registers accordingly
                CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "C1FLTOBJ[%d]=0x%08"PRIx32"\n", idx, filter->fltobj);
                CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "C1MASK[%d]=0x%08"PRIx32"\n", idx, filter->mask);

                mcp251718fd_spi_write_word(C1FLTOBJ(idx), filter->fltobj);
                mcp251718fd_spi_write_word(C1MASK(idx), filter->mask);
            }
        }
    } else {
        CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Setting a global allow-all filter\n");
        // Set filter 0 to match-all and direct frames to the RX FIFO
        filter_control[0] = 0x81U;
        mcp251718fd_spi_write_word(C1FLTOBJ(0), 0);
        mcp251718fd_spi_write_word(C1MASK(0), 0);
    }

    // Enable the appropriate filters
    for (uint32_t i = 0; i < 8U; i++) {
        CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "C1FLTCON[%d]=0x%08"PRIx32"\n", i, filter_control[i]);
        mcp251718fd_spi_write_word(C1FLTCON(i), filter_control[i]);
    }

    init_structures(self);

    self->recv_errors = recv_errors;
    self->mode = mode;
    self->seq_bad = 0;
    self->txqua_bad = 0;
    self->txqsta_bad = 0;

    // Enable SPI interrupt on Pico pin, level sensitive, low (all MCP2517/18FD interrupt
    // pins are active low), and will add in a callback into the vector table.
    //
    // Note that this might need to change if other software in the Pico is using the GPIO
    // interrupt handler.
    //
    // Call will enable GPIO interrupts, make them interrupt the timer etc.
    // There is an issue because all the GPIO interrupts of this bank go through the same vector,
    // which is shared with a stub that allows MicroPython functions to run as ISRs. A hand-off mechanism is
    // used for MicroPython function ISRs (see machine_pin.c).
    //
    // Default IRQ priority is 0x80 (i.e. 2, where 0 is the highest and 3 is the lowest).
    irq_set_priority(SPI_IRQ_GPIO, SPI_GPIO_IRQ_PRIORITY);
    // Allow the interrupts at source
    gpio_set_irq_enabled(SPI_IRQ_GPIO, LEVEL_SENSITIVE_LOW, true);

    ENABLE_GPIO_INTERRUPTS();

    return self;
}

STATIC mp_obj_t rp2_can_send_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_frame,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_fifo,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canframe_obj_t *frame = args[0].u_obj;
    bool fifo = args[1].u_bool;

    if(!MP_OBJ_IS_TYPE(frame, &rp2_canframe_type)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "CANFrame expected"));
    }

    CAN_ASSERT(!irq_locked, "A6")
    DISABLE_GPIO_INTERRUPTS();
    bool queued = mcp251718fd_send_frame(self, frame, fifo, &self->tx_queue, &self->tx_fifo);
    ENABLE_GPIO_INTERRUPTS();

    if (!queued) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in transmit queue"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_send_frame_obj, 1, rp2_can_send_frame);

STATIC mp_obj_t rp2_can_send_frames(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_frames,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_fifo,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_list_t *frames = args[0].u_obj;
    bool fifo = args[1].u_bool;

    if(!MP_OBJ_IS_TYPE(frames, &mp_type_list)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "List of CAN frames expected"));
    }
    // Quick check to see that the list contains only CAN frames
    for (uint32_t i = 0; i < frames->len; i++) {
        rp2_canframe_obj_t *frame = frames->items[i];
        if(!MP_OBJ_IS_TYPE(frame, &rp2_canframe_type)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "CANFrame expected"));
        }
    }
    // Check there is room for the total number of frames, in the queue or in the software FIFO
    // so that all the frames are queued or none are
    if (fifo) {
        if (self->tx_queue.fifo_slot == MCP251718FD_TX_QUEUE_SIZE) {
            // No existing FIFO frame in the transmit queue
            if (self->tx_queue.num_free_slots < 1U) {
                nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in transmit queue for the first frame"));
            }
            if (frames->len - 1U > self->tx_fifo.num_free_slots) {
                nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in transmit FIFO for the remaining frames"));
            }
        }
        else {
            if (frames->len > self->tx_fifo.num_free_slots) {
                nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in transmit FIFO for all the frames"));
            }
        }
    }
    else {
        if (frames->len > self->tx_queue.num_free_slots) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in transmit queue for all the frames"));
        }
    }

    // Now can queue the frames because there is space and everything is of the right type
    uint32_t queued_frames = 0;
    for (uint32_t i = 0; i < frames->len; i++) {
        rp2_canframe_obj_t *frame = frames->items[i];
        CAN_ASSERT(!irq_locked, "A7")
        DISABLE_GPIO_INTERRUPTS();
        bool result = mcp251718fd_send_frame(self, frame, fifo, &self->tx_queue, &self->tx_fifo);
        if (result) {
            queued_frames++;
        }
        ENABLE_GPIO_INTERRUPTS();
    }

    // TODO could update API to return the number of queued frames and allow partial queueing of a list
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_send_frames_obj, 1, rp2_can_send_frames);

STATIC TIME_CRITICAL void pop_rx_frame(rp2_can_obj_t *self, mp_obj_list_t *list, uint32_t i)
{
    // This must be called with interrupts disabled
    rp2_canframe_obj_t *frame;
    rp2_canerror_obj_t *error_frame;

    // Pop the front of the receive FIFO
    self->rx_fifo.free++;
    uint8_t idx = self->rx_fifo.head_idx++;
    if (self->rx_fifo.head_idx == RX_FIFO_SIZE) {
        self->rx_fifo.head_idx = 0;
    }

    // Fill in the list with an appropriate item
    // NB: don't use a switch because we don't want the compiler to generate switch tables in ROM that
    // then lives in serial flash
    canevent_type_t ev = self->rx_fifo.rx_events[idx].event_type;

    if (ev == EVENT_TYPE_OVERFLOW) {
        list->items[i] = mp_const_none;
    }
    else if (ev == EVENT_TYPE_CAN_ERROR) {
        error_frame = m_new_obj(rp2_canerror_obj_t);
        error_frame->base.type = &rp2_canerror_type;
        error_frame->c1bdiag1 = self->rx_fifo.rx_events[idx].info.c1bdiag1;
        error_frame->timestamp = self->rx_fifo.rx_events[idx].timestamp;

        list->items[i] = error_frame;
    }
    else if (ev == EVENT_TYPE_RECEIVED_FRAME) {
        // Create a new CANFrame object
        frame = m_new_obj(rp2_canframe_obj_t);
        frame->base.type = &rp2_canframe_type;
        // Fill in the details from the event
        frame->can_id = self->rx_fifo.rx_events[idx].info.rx_frame.canid;
        frame->timestamp = self->rx_fifo.rx_events[idx].timestamp;
        frame->tag = 0;
        frame->timestamp_valid = true;
        frame->id_filter = self->rx_fifo.rx_events[idx].info.rx_frame.id_filter;
        frame->remote = self->rx_fifo.rx_events[idx].info.rx_frame.remote;
        frame->dlc = self->rx_fifo.rx_events[idx].info.rx_frame.dlc;
        frame->payload[0] = self->rx_fifo.rx_events[idx].info.rx_frame.payload[0];
        frame->payload[1] = self->rx_fifo.rx_events[idx].info.rx_frame.payload[1];

        list->items[i] = frame;
    }
    else {
        // This should never happen but fill in something anyway
        list->items[i] = mp_const_none;
    }
}

// Pop a transmit event (if there's room) into a buffer that has n bytes spare
STATIC TIME_CRITICAL uint32_t pop_rx_frame_as_bytes(rp2_can_obj_t *self, uint8_t *buf, uint32_t n)
{
    // This must be called with interrupts disabled

    // Frame is a fixed size
    if (n >= NUM_FRAME_BYTES) {
        // Pop the front of the transmit event FIFO
        self->rx_fifo.free++;
        uint8_t idx = self->rx_fifo.head_idx++;
        if (self->rx_fifo.head_idx == TX_EVENT_FIFO_SIZE) {
            self->rx_fifo.head_idx = 0;
        }

        buf[0] = self->rx_fifo.rx_events[idx].event_type;                                       // Flags byte
        BIG_ENDIAN_BUF(buf + 1U, self->rx_fifo.rx_events[idx].timestamp);           // Timestamp

        canevent_type_t ev = self->rx_fifo.rx_events[idx].event_type;
        if (ev == EVENT_TYPE_OVERFLOW) {
            // Pack out the rest of the bytes with the overflow counts
            BIG_ENDIAN_BUF(buf + 7U, self->rx_fifo.rx_events[idx].info.overflow_cnt.frame_cnt);
            BIG_ENDIAN_BUF(buf + 11U, self->rx_fifo.rx_events[idx].info.overflow_cnt.error_cnt);
        }
        else if (ev == EVENT_TYPE_CAN_ERROR) {
            // Pack out the rest of the bytes with the details of the error
            BIG_ENDIAN_BUF(buf + 7U, self->rx_fifo.rx_events[idx].info.c1bdiag1);
        }
        else if (ev == EVENT_TYPE_RECEIVED_FRAME) {
            // Pack out the rest of the bytes with the frame details
            // Add flag info to indicate a remote frame
            buf[0] |= self->rx_fifo.rx_events[idx].info.rx_frame.remote ? 0x80U : 0x00U;
            // DLC, ID filter hit, timestamp, CAN ID, payload
            buf[5] = self->rx_fifo.rx_events[idx].info.rx_frame.dlc;
            buf[6] = self->rx_fifo.rx_events[idx].info.rx_frame.id_filter;
            BIG_ENDIAN_BUF(buf + 7U, self->rx_fifo.rx_events[idx].info.rx_frame.canid);
            for (size_t i = 0; i < 8U; i++) {
                buf[11 + i] = *((uint8_t *) (self->rx_fifo.rx_events[idx].info.rx_frame.payload) + i);
            }
        }
        return NUM_FRAME_BYTES;
    }
    else {
        return 0;
    }
}

STATIC mp_obj_t rp2_can_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_limit,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = RX_FIFO_SIZE}},
        {MP_QSTR_as_bytes, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // rx_fifo.free is safe to access outside the ISR because it's an atomic word and can only decrease so num_frames can only increase
    uint32_t num_frames = RX_FIFO_SIZE - self->rx_fifo.free;
    uint32_t limit = args[0].u_int;
    bool as_bytes = args[1].u_bool;

    if (limit > num_frames) {
        limit = num_frames;
    }

    if (as_bytes) {
        // Temporary buffer to store some frames
        uint8_t buf[255];

        size_t remaining = sizeof(buf);
        size_t n = 0;

        // Pull frames from the FIFO up to a limit, keeping track of the bytes added
        for (uint32_t i = 0; i < limit; i++) {
            CAN_ASSERT(!irq_locked, "A8")
            DISABLE_GPIO_INTERRUPTS();
            size_t added = pop_rx_frame_as_bytes(self, buf + n, remaining);
            ENABLE_GPIO_INTERRUPTS();
            if (added) {
                n += added;
                remaining -= added;
            }
            else {
                break;
            }
        }
        // Return bytes for the block of frames (might be zero)
        return make_mp_bytes(buf, n);
    }
    else {
        // Frames that will be pulled are the minimum of the limit or the number of frames in the RX FIFO
        // Will return an empty list if there are no frames
        mp_obj_list_t *list = mp_obj_new_list(limit, NULL);

        // Pull frames from the FIFO up to a limit
        for (uint32_t i = 0; i < limit; i++) {
            CAN_ASSERT(!irq_locked, "A9")
            DISABLE_GPIO_INTERRUPTS();
            pop_rx_frame(self, list, i);
            ENABLE_GPIO_INTERRUPTS();
        }
        return list;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_recv_obj, 1, rp2_can_recv);

// Return number of messages waiting in the RX FIFO.`
STATIC mp_obj_t rp2_can_recv_pending(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;

    // It's OK to access this with interrupt concurrency because it's an atomic word
    uint32_t n = RX_FIFO_SIZE - self->rx_fifo.free;

    return MP_OBJ_NEW_SMALL_INT(n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_recv_pending_obj, rp2_can_recv_pending);


STATIC TIME_CRITICAL void pop_tx_event(rp2_can_obj_t *self, mp_obj_list_t *list, uint32_t i)
{
    // This must be called with interrupts disabled

    // Pop the front of the transmit event FIFO
    self->event_fifo.free++;
    uint8_t idx = self->event_fifo.head_idx++;
    if (self->event_fifo.head_idx == TX_EVENT_FIFO_SIZE) {
        self->event_fifo.head_idx = 0;
    }

    // Create a new event object (a tuple of event type, tag and timestamp)
    mp_obj_tuple_t *event = mp_obj_new_tuple(3U, NULL);

    event->items[0] = mp_obj_new_int_from_uint(self->event_fifo.events[idx].event_type);
    event->items[1] = mp_obj_new_int_from_uint(self->event_fifo.events[idx].info.generic);
    event->items[2] = mp_obj_new_int_from_uint(self->event_fifo.events[idx].timestamp);

    // This event is the i'th entry in the tuple of events being returned
    list->items[i] = event;
}

// Pop a transmit event (if there's room) into a buffer that has n bytes spare
STATIC TIME_CRITICAL uint32_t pop_tx_event_as_bytes(rp2_can_obj_t *self, uint8_t *buf, uint32_t n)
{
    // This must be called with interrupts disabled

    // Transmit event is:
    // Byte 0: flags, bit 7 = overflow indicator (bytes 1-8 undefined)
    // Bytes 1-4: tag (in little endian format)
    // Bytes 5-8: timestamp (in little endian format)

    // Transmit event is exactly 9 bytes
    if (n >= 9U) {
        // Pop the front of the transmit event FIFO
        self->event_fifo.free++;
        uint8_t idx = self->event_fifo.head_idx++;
        if (self->event_fifo.head_idx == TX_EVENT_FIFO_SIZE) {
            self->event_fifo.head_idx = 0;
        }

        // Access the relevant parameter via a generic union member
        uint32_t generic = self->event_fifo.events[idx].info.generic;
        uint32_t timestamp = self->event_fifo.events[idx].timestamp;

        // Set flags for the type of event
        buf[0] = self->event_fifo.events[idx].event_type;

        BIG_ENDIAN_BUF(buf + 1U, generic);
        BIG_ENDIAN_BUF(buf + 5U, timestamp);
        return NUM_EVENT_BYTES;
    }
    else {
        return 0;
    }
}

STATIC mp_obj_t rp2_can_recv_tx_events(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            {MP_QSTR_limit,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = TX_EVENT_FIFO_SIZE}},
            {MP_QSTR_as_bytes, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // tx_event_fifo.free is safe to access outside the ISR because it's an atomic word and can only decrease so num_events can only increase
    uint32_t num_events = TX_EVENT_FIFO_SIZE - self->event_fifo.free;
    uint32_t limit = args[0].u_int;
    bool as_bytes = args[1].u_bool;

    if (limit > num_events) {
        limit = num_events;
    }

    if (as_bytes) {
        uint8_t buf[128];
        size_t remaining = sizeof(buf);
        size_t n = 0;

        // Pull events from the FIFO up to a limit
        for (uint32_t i = 0; i < limit; i++) {
            CAN_ASSERT(!irq_locked, "AA")
            DISABLE_GPIO_INTERRUPTS();
            size_t added = pop_tx_event_as_bytes(self, buf + n, remaining);
            ENABLE_GPIO_INTERRUPTS();
            if (added) {
                n += added;
                remaining -= added;
            }
            else {
                break;
            }
        }
        // Return bytes for the block of transmit events (might be zero)
        return make_mp_bytes(buf, n);
    }
    else {
        // Events that will be pulled are the minimum of the limit or the number of events in the TX event FIFO
        // Will return an empty tuple constant if there are no events
        mp_obj_list_t *list = mp_obj_new_list(limit, NULL);

        // Pull events from the FIFO up to a limit
        for (uint32_t i = 0; i < limit; i++) {
            CAN_ASSERT(!irq_locked, "AB")
            DISABLE_GPIO_INTERRUPTS();
            pop_tx_event(self, list, i);
            ENABLE_GPIO_INTERRUPTS();
        }
        return list;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_recv_tx_events_obj, 1, rp2_can_recv_tx_events);

// Return number of events waiting in the TX event FIFO.`
STATIC mp_obj_t rp2_can_recv_tx_events_pending(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;

    // It's OK to access this with interrupt concurrency because it's an atomic word
    uint32_t n = RX_FIFO_SIZE - self->event_fifo.free;

    return MP_OBJ_NEW_SMALL_INT(n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_recv_tx_events_pending_obj, rp2_can_recv_tx_events_pending);

// Return number of messages waiting in the RX FIFO.
STATIC mp_obj_t rp2_can_get_status(mp_obj_t self_in)
{
    // Not used
    // rp2_can_obj_t *self = self_in;
    uint32_t trec = mcp251718fd_get_trec();

    // Returns a tuple of:
    // bool: is Bus-off
    // bool: is Error Passive
    // int: TEC
    // int: REC
    mp_obj_tuple_t *tuple = mp_obj_new_tuple(4U, NULL);
    tuple->items[0] = (trec & (1U << 21)) ? mp_const_true : mp_const_false;
    tuple->items[1] = (trec & (3U << 19)) ? mp_const_true : mp_const_false;
    tuple->items[2] = MP_OBJ_NEW_SMALL_INT((trec >> 8) & 0xffU);
    tuple->items[3] = MP_OBJ_NEW_SMALL_INT(trec & 0xffU);

    return tuple;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_status_obj, rp2_can_get_status);

// Return diagnostic values
STATIC mp_obj_t rp2_can_get_diagnostics(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;
    // Returns a tuple of:
    //
    // integer: number of times SEQ was corrupted
    // integer: number of times TXQUA was read as corrupted
    // integer: number of times TXQSTA was read as corrupted
    //
    // Other data may be added here to help diagnose faults
    mp_obj_tuple_t *tuple = mp_obj_new_tuple(3U, NULL);
    tuple->items[0] = MP_OBJ_NEW_SMALL_INT(self->seq_bad);
    tuple->items[1] = MP_OBJ_NEW_SMALL_INT(self->txqua_bad);
    tuple->items[2] = MP_OBJ_NEW_SMALL_INT(self->txqsta_bad);

    return tuple;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_diagnostics_obj, rp2_can_get_diagnostics);

// Return the timestamp counter
// (Typically used to convert timestamps to time-of-day)
STATIC mp_obj_t rp2_can_get_time(mp_obj_t self_in)
{
    // Not used
    // rp2_can_obj_t *self = self_in;

    return mp_obj_new_int_from_uint(mcp251718fd_get_timebase());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_time_obj, rp2_can_get_time);

// Return the timestamp counter resolution in ticks per second
STATIC mp_obj_t rp2_can_get_time_hz(mp_obj_t self_in)
{
    // Not used
    // rp2_can_obj_t *self = self_in;

    return mp_obj_new_int_from_uint(1000000U);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_time_hz_obj, rp2_can_get_time_hz);

// Return number of frame slots free in the transmit or FIFO queues
STATIC mp_obj_t rp2_can_get_send_space(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_fifo,    MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool fifo = args[0].u_bool;

    return fifo ? MP_OBJ_NEW_SMALL_INT(self->tx_fifo.num_free_slots) : MP_OBJ_NEW_SMALL_INT(self->tx_queue.num_free_slots);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_get_send_space_obj, 1, rp2_can_get_send_space);

// Set the conditions for triggering an edge on the trigger pin
STATIC mp_obj_t rp2_can_set_trigger(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            {MP_QSTR_on_error,      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
            {MP_QSTR_on_canid,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},   // A specific CAN ID
            {MP_QSTR_as_bytes,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},   // A block of bytes
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool on_error = args[0].u_bool;
    rp2_canid_obj_t *on_canid = args[1].u_obj;
    mp_obj_t as_bytes = args[2].u_obj;

    if (as_bytes != MP_OBJ_NULL) {
        // Trigger can be set directly but the ID trigger is then not valid
        if (!MP_OBJ_IS_STR_OR_BYTES(as_bytes)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Trigger must be bytes"));
        }
        if (on_canid != MP_OBJ_NULL) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Cannot set a binary trigger and an ID trigger"));
        }

        uint8_t trigger_buf[27];
        uint32_t len = (uint8_t)copy_mp_bytes(as_bytes, (uint8_t *)trigger_buf, sizeof(trigger_buf));
        if (len != sizeof(trigger_buf)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Trigger must be %d bytes", sizeof(trigger_buf)));
        }
        self->triggers[0].on_error = (trigger_buf[0] & 0x80U) != 0;

        uint8_t *can_payload_mask = (uint8_t *)self->triggers[0].can_payload_mask;
        uint8_t *can_payload_match = (uint8_t *)self->triggers[0].can_payload_match;
        for (uint8_t i = 0; i < 8U; i++) {
            can_payload_mask[i] = trigger_buf[i + 11U];
            can_payload_match[i] = trigger_buf[i + 19U];
        }
        self->triggers[0].can_dlc_mask = trigger_buf[9];
        self->triggers[0].can_dlc_match = trigger_buf[10];
        self->triggers[0].can_id_mask = BIG_ENDIAN_WORD(trigger_buf + 1U);
        self->triggers[0].can_id_match = BIG_ENDIAN_WORD(trigger_buf + 5U);

        self->triggers[0].on_rx = true;
        self->triggers[0].enabled = true;
    }
    else if (on_canid != MP_OBJ_NULL) {
        if (!MP_OBJ_IS_TYPE(on_canid, &rp2_canid_type)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "on_canid must be of type CANID"));
        }
        if (as_bytes != MP_OBJ_NULL) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Cannot set an ID trigger and a binary trigger"));
        }
        // Set masks to allow all payloads and sizes
        self->triggers[0].can_payload_match[0] = 0;
        self->triggers[0].can_payload_match[1] = 0;
        self->triggers[0].can_payload_mask[0] = 0;
        self->triggers[0].can_payload_mask[1] = 0;
        self->triggers[0].can_dlc_match = 0;
        self->triggers[0].can_dlc_mask = 0;
        // Set ID trigger
        self->triggers[0].can_id_mask = 0xffffffffU;
        self->triggers[0].can_id_match = on_canid->can_id;

        self->triggers[0].on_rx = true;
        self->triggers[0].enabled = true;
    }
    else {
        if (on_error) {
            self->triggers[0].on_error = on_error;
            self->triggers[0].enabled = true;
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_set_trigger_obj, 1, rp2_can_set_trigger);

// Clear the trigger from operating
STATIC mp_obj_t rp2_can_clear_trigger(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;

    self->triggers[0].enabled = false;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_clear_trigger_obj, rp2_can_clear_trigger);

// Put a pulse on the trigger pin
STATIC mp_obj_t rp2_can_pulse_trigger(mp_obj_t self_in)
{
    // Not used
    // rp2_can_obj_t *self = self_in;

    // Ensure pulse is long enough for even a slow logic analyzer (e.g. 20MHz) to see
    TRIG_SET();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    TRIG_CLEAR();
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_pulse_trigger_obj, rp2_can_pulse_trigger);

//////////////////////////////////////// TEST FUNCTIONS /////////////////////////////////////////

STATIC mp_obj_t rp2_can_test_irq_init(void)
{
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Enabling GPIO IRQ\n");
    gpio_set_irq_enabled(SPI_IRQ_GPIO, EDGE_SENSITIVE_RISING, true);
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Enabled GPIO IRQ and vector\n");
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Set IRQ handler\n");
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "IRQ enabled\n");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_irq_init_fun_obj, rp2_can_test_irq_init);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_irq_init_obj, MP_ROM_PTR(&rp2_can_test_irq_init_fun_obj));

STATIC mp_obj_t rp2_can_test_irq_enable(void)
{
    DISABLE_GPIO_INTERRUPTS();
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "DISABLE_GPIO_INTERRUPTS() called\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_irq_enable_fun_obj, rp2_can_test_irq_enable);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_irq_enable_obj, MP_ROM_PTR(&rp2_can_test_irq_enable_fun_obj));

STATIC mp_obj_t rp2_can_test_irq_disable(void)
{
    ENABLE_GPIO_INTERRUPTS();
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "ENABLE_GPIO_INTERRUPTS() called\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_irq_disable_fun_obj, rp2_can_test_irq_disable);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_irq_disable_obj, MP_ROM_PTR(&rp2_can_test_irq_disable_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_init(void)
{
    DISABLE_GPIO_INTERRUPTS();
    pico_pin_init();
    ENABLE_GPIO_INTERRUPTS();
    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "VTOR=0x%08"PRIx32"\n", scb_hw->vtor);
    for(int i = -16; i < 0x40; i++) {
        CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "VTOR[%d]=0x%08"PRIx32"\n", i, vtor[i + 16]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_spi_init_fun_obj, rp2_can_test_spi_init);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_init_obj, MP_ROM_PTR(&rp2_can_test_spi_init_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_set(void)
{
    SPI_SELECT();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_spi_set_fun_obj, rp2_can_test_spi_set);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_set_obj, MP_ROM_PTR(&rp2_can_test_spi_set_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_deselect(void)
{
    SPI_DESELECT();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_spi_deselect_fun_obj, rp2_can_test_spi_deselect);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_deselect_obj, MP_ROM_PTR(&rp2_can_test_spi_deselect_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_write_word(mp_obj_t addr_obj, mp_obj_t word_obj)
{
    uint32_t addr = mp_obj_get_int(addr_obj);
    uint32_t word = mp_obj_get_int(word_obj);

    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Writing word=0x%08"PRIx32"\n", word);
    DISABLE_GPIO_INTERRUPTS();
    mcp251718fd_spi_write_word(addr | 0x2000, word);
    ENABLE_GPIO_INTERRUPTS();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(rp2_can_test_spi_write_word_fun_obj, rp2_can_test_spi_write_word);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_write_word_obj, MP_ROM_PTR(&rp2_can_test_spi_write_word_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_read_word(mp_obj_t addr_obj)
{
    uint32_t addr = mp_obj_get_int(addr_obj);

    DISABLE_GPIO_INTERRUPTS();
    uint32_t result = mcp251718fd_spi_read_word(addr);
    ENABLE_GPIO_INTERRUPTS();

    CAN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Read word=0x%08"PRIx32"\n", result);
    return mp_obj_new_int_from_ull(result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_test_spi_read_word_fun_obj, rp2_can_test_spi_read_word);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_read_word_obj, MP_ROM_PTR(&rp2_can_test_spi_read_word_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_read_words(mp_obj_t addr_obj)
{
    uint32_t words[4];
    uint32_t addr = mp_obj_get_int(addr_obj);

    DISABLE_GPIO_INTERRUPTS();
    mcp251718fd_spi_read_words(addr, words, 4U);
    ENABLE_GPIO_INTERRUPTS();

    mp_obj_tuple_t *tuple = mp_obj_new_tuple(4U, NULL);
    tuple->items[0] = mp_obj_new_int_from_ull(words[0]);
    tuple->items[1] = mp_obj_new_int_from_ull(words[1]);
    tuple->items[2] = mp_obj_new_int_from_ull(words[2]);
    tuple->items[3] = mp_obj_new_int_from_ull(words[3]);

    return tuple;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_test_spi_read_words_fun_obj, rp2_can_test_spi_read_words);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_read_words_obj, MP_ROM_PTR(&rp2_can_test_spi_read_words_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_write_words(mp_obj_t addr_obj)
{
    uint32_t addr = mp_obj_get_int(addr_obj);
    uint32_t words[4] = {0xdeadbeefU, 0xcafef00dU, 0x01e551caU, 0x01020304U};

    DISABLE_GPIO_INTERRUPTS();
    mcp251718fd_spi_write_4words(addr, words);
    ENABLE_GPIO_INTERRUPTS();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_test_spi_write_words_fun_obj, rp2_can_test_spi_write_words);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_write_words_obj, MP_ROM_PTR(&rp2_can_test_spi_write_words_fun_obj));


STATIC void rp2_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
     rp2_can_obj_t *self = self_in;

    uint32_t trec = mcp251718fd_get_trec();
    uint32_t timebase = mcp251718fd_get_timebase();
    uint32_t tec = (trec >> 8) & 0xffU;
    uint32_t rec = trec & 0xffU;

    // Show the bus off status, the error passive status, TEC, REC, the time, and the baud rate settings
    mp_printf(print, "CAN(mode=");
    if (self->mode == CAN_MODE_OFFLINE) {
        mp_printf(print, "CAN_MODE_OFFLINE");
    }
    else if (self->mode == CAN_MODE_ACK_ONLY) {
        mp_printf(print, "CAN_MODE_ACK_ONLY");
    }
    else if (self->mode == CAN_MODE_LISTEN_ONLY) {
        mp_printf(print, "CAN_MODE_LISTEN_ONLY");
    }
    else if (self->mode == CAN_MODE_NORMAL) {
        mp_printf(print, "CAN_MODE_NORMAL");
    }
    else {
        mp_printf(print, "?");
    }
    if (self->recv_errors) {
        mp_printf(print, ", recv_errors=True");
    }

    mp_printf(print, ", time=%lu, TEC=%d, REC=%d", timebase, tec, rec);

    // Error states
    if (trec & (1U << 21))  {
        mp_printf(print, ", Bus Off");
    }
    if (trec & (3U << 19)) {
        mp_printf(print, ", Error Passive");
    }
    if (trec & (1U << 16)) {
        mp_printf(print, ", Warn");
    }

    mp_printf(print, ")");
}

STATIC const mp_map_elem_t rp2_can_locals_dict_table[] = {
    ////// Instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_frame), (mp_obj_t)&rp2_can_send_frame_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_frames), (mp_obj_t)&rp2_can_send_frames_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv), (mp_obj_t)&rp2_can_recv_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv_pending), (mp_obj_t)&rp2_can_recv_pending_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv_tx_events), (mp_obj_t)&rp2_can_recv_tx_events_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv_tx_events_pending), (mp_obj_t)&rp2_can_recv_tx_events_pending_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_status), (mp_obj_t)&rp2_can_get_status_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_diagnostics), (mp_obj_t)&rp2_can_get_diagnostics_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_send_space), (mp_obj_t)&rp2_can_get_send_space_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_time), (mp_obj_t)&rp2_can_get_time_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_time_hz), (mp_obj_t)&rp2_can_get_time_hz_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_trigger), (mp_obj_t)&rp2_can_set_trigger_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear_trigger), (mp_obj_t)&rp2_can_clear_trigger_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pulse_trigger), (mp_obj_t)&rp2_can_pulse_trigger_obj },

    ////// Static methods
    // Test methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_irq_init), (mp_obj_t)&rp2_can_test_irq_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_irq_enable), (mp_obj_t)&rp2_can_test_irq_enable_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_irq_disable), (mp_obj_t)&rp2_can_test_irq_disable_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_init), (mp_obj_t)&rp2_can_test_spi_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_set), (mp_obj_t)&rp2_can_test_spi_set_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_deselect), (mp_obj_t)&rp2_can_test_spi_deselect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_write_word), (mp_obj_t)&rp2_can_test_spi_write_word_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_read_word), (mp_obj_t)&rp2_can_test_spi_read_word_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_read_words), (mp_obj_t)&rp2_can_test_spi_read_words_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_test_spi_write_words), (mp_obj_t)&rp2_can_test_spi_write_words_obj },

    ////// Class constants
    // Pre-defined bit rates
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_500K_75), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_500K_75) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_250K_75), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_250K_75) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_125K_75), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_125K_75) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_1M_75), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_1M_75) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_500K_50), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_500K_50) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_250K_50), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_250K_50) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_125K_50), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_125K_50) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_1M_50), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_1M_50) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_2M_50), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_2M_50) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_4M_90), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_4M_90) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_2_5M_75), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_2_5M_75) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_2M_80), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_2M_80) },

    // Mode constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_NORMAL), MP_OBJ_NEW_SMALL_INT(CAN_MODE_NORMAL) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LISTEN_ONLY), MP_OBJ_NEW_SMALL_INT(CAN_MODE_LISTEN_ONLY) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ACK_ONLY), MP_OBJ_NEW_SMALL_INT(CAN_MODE_ACK_ONLY) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_OFFLINE), MP_OBJ_NEW_SMALL_INT(CAN_MODE_OFFLINE) },

    // Configuration constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_RX_FIFO_SIZE), MP_OBJ_NEW_SMALL_INT(RX_FIFO_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_FIFO_SIZE), MP_OBJ_NEW_SMALL_INT(MCP251718FD_TX_FIFO_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_QUEUE_SIZE), MP_OBJ_NEW_SMALL_INT(MCP251718FD_TX_QUEUE_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_EVENT_FIFO_SIZE), MP_OBJ_NEW_SMALL_INT(TX_EVENT_FIFO_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_OVERFLOW), MP_OBJ_NEW_SMALL_INT(EVENT_TYPE_OVERFLOW) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_CAN_ERROR), MP_OBJ_NEW_SMALL_INT(EVENT_TYPE_CAN_ERROR) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_TRANSMITTED_FRAME), MP_OBJ_NEW_SMALL_INT(EVENT_TYPE_TRANSMITTED_FRAME) },
};
STATIC MP_DEFINE_CONST_DICT(rp2_can_locals_dict, rp2_can_locals_dict_table);

const mp_obj_type_t rp2_can_type = {
    { &mp_type_type },
    .name = MP_QSTR_CAN,
    .print = rp2_can_print,
    .make_new = rp2_can_make_new,
    .locals_dict = (mp_obj_t)&rp2_can_locals_dict,
};
////////////////////////////////////// End of CAN class //////////////////////////////////////

////////////////////////////////// Start of CANFrame class ///////////////////////////////////
STATIC mp_obj_t rp2_canframe_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_canid,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_data,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_remote,    MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_tag,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_dlc,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canid_obj_t *canid = args[0].u_obj;
    mp_obj_t data = args[1].u_obj;
    bool remote = args[2].u_bool;
    uint32_t tag = args[3].u_int;
    bool dlc_set = args[4].u_int != -1;
    uint8_t dlc = args[4].u_int;

    if (!MP_OBJ_IS_TYPE(canid, &rp2_canid_type)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "canid must be of type CANID"));
    }
    if (dlc_set && dlc > 15) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "dlc must be 0..15"));
    }
    if (dlc_set && !remote && data == MP_OBJ_NULL && dlc > 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "dlc must be 0 if no payload"));
    }

    rp2_canframe_obj_t *self = m_new_obj(rp2_canframe_obj_t);
    self->base.type = &rp2_canframe_type;

    self->remote = remote;
    if(data == MP_OBJ_NULL) {
        if (remote) {
            self->dlc = dlc_set ? dlc : 0;
        }
        else {
            self->dlc = 0;
        }
    }
    else {
        uint32_t buf[2];
        if(remote) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Remote frames cannot have a payload"));
        }
        uint8_t len = (uint8_t)copy_mp_bytes(data, (uint8_t *)buf, 8U);
        // If there are insufficient bytes to match the DLC then this is an error
        if (dlc_set && len < 8U && dlc > len) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "dlc exceeds data length"));
        }
        self->dlc = dlc_set ? dlc : len;

        // Little-endian CPU, little endian buffering in MCP2517/18FD
        self->payload[0] = buf[0];
        self->payload[1] = buf[1];
    }
    self->can_id = canid->can_id;
    self->timestamp_valid = false;
    self->id_filter = 0;
    self->tag = tag;

    return self;
}

//////////////////////////// Static class method to create a list of CANFrame instances ///////////////////////////
STATIC mp_obj_t rp2_canframe_from_bytes(mp_obj_t frames)
{
    if (!MP_OBJ_IS_STR_OR_BYTES(frames)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "String/bytes parameter expected"));
    }
    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    rp2_buf_get_for_send(frames, &bufinfo, data);

    if ((bufinfo.len % NUM_FRAME_BYTES) != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Frames must be a multiple of %d bytes", NUM_FRAME_BYTES));
    }
    size_t num_frames = bufinfo.len / 19U;
    uint8_t *buf_ptr = bufinfo.buf;
    mp_obj_list_t *list = mp_obj_new_list(num_frames, NULL);

    for (uint32_t i = 0; i < num_frames; i++) {
        rp2_canframe_obj_t *self = m_new_obj(rp2_canframe_obj_t);
        self->base.type = &rp2_canframe_type;

        // Data not known at creation time
        self->timestamp_valid = false;
        self->id_filter = 0;                // Only received frames have this filled in

        // Fill in data from the buffer
        self->remote = (buf_ptr[0] & 0x01U) != 0;
        self->dlc = buf_ptr[1] & 0x0fU;
        self->tag = BIG_ENDIAN_WORD(buf_ptr + 3U);
        self->can_id = BIG_ENDIAN_WORD(buf_ptr + 7);

        ((uint8_t *)(self->payload))[0] = buf_ptr[11];
        ((uint8_t *)(self->payload))[1] = buf_ptr[12];
        ((uint8_t *)(self->payload))[2] = buf_ptr[13];
        ((uint8_t *)(self->payload))[3] = buf_ptr[14];
        ((uint8_t *)(self->payload))[4] = buf_ptr[15];
        ((uint8_t *)(self->payload))[5] = buf_ptr[16];
        ((uint8_t *)(self->payload))[6] = buf_ptr[17];
        ((uint8_t *)(self->payload))[7] = buf_ptr[18];

        list->items[i] = self;
        buf_ptr += NUM_FRAME_BYTES;
    }

    return list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_from_bytes_fun_obj, rp2_canframe_from_bytes);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_canframe_from_bytes_obj, MP_ROM_PTR(&rp2_canframe_from_bytes_fun_obj));

// Get the payload as bytes
STATIC mp_obj_t rp2_canframe_get_data(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;
    uint32_t len;

    if (self->remote) {
        len = 0;
    }
    else {
        if (self->dlc > 8U) {
            len = 8U;
        }
        else {
            len = self->dlc;
        }
    }
    return make_mp_bytes((uint8_t *)(self->payload), len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_data_obj, rp2_canframe_get_data);

// Get the payload as bytes
STATIC mp_obj_t rp2_canframe_get_dlc(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(self->dlc);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_dlc_obj, rp2_canframe_get_dlc);

// Get the frame's tag
STATIC mp_obj_t rp2_canframe_get_tag(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return mp_obj_new_int_from_uint(self->tag);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_tag_obj, rp2_canframe_get_tag);

// Returns the frame's timestamp, or None if no timestamp valid
STATIC mp_obj_t rp2_canframe_get_timestamp(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    // An ISR can set timestamp_valid but since it is a boolean will not need to disable interrupts
    if (self->timestamp_valid) {
        return mp_obj_new_int_from_uint(self->timestamp);
    }
    else {
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_timestamp_obj, rp2_canframe_get_timestamp);

// Returns the ID acceptance filter that allowed through the frame
STATIC mp_obj_t rp2_canframe_get_index(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(self->id_filter);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_index_obj, rp2_canframe_get_index);

// Returns True if the frame has an extended ID, False otherwise
STATIC mp_obj_t rp2_canframe_is_extended(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return self->can_id & (1U << 29) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_is_extended_obj, rp2_canframe_is_extended);

// Returns True if the frame is a remote frame, False otherwise
STATIC mp_obj_t rp2_canframe_is_remote(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return self->remote ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_is_remote_obj, rp2_canframe_is_remote);

// Get a CANID instance representing the frame's CAN ID
STATIC mp_obj_t rp2_canframe_get_canid(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;
    rp2_canid_obj_t *canid = m_new_obj(rp2_canid_obj_t);
    canid->base.type = &rp2_canid_type;
    canid->can_id = self->can_id;

    return canid;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_canid_obj, rp2_canframe_get_canid);

// Get a numeric value for the CAN ID, with 11-bit IDs in the range 0..7ff
STATIC mp_obj_t rp2_canframe_get_arbitration_id(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    uint32_t arbitration_id = self->can_id & (1U << 29) ? self->can_id & 0x1fffffffU : (self->can_id >> 18) & 0x7ffU;

    return MP_OBJ_NEW_SMALL_INT(arbitration_id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_arbitration_id_obj, rp2_canframe_get_arbitration_id);

STATIC void rp2_canframe_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canframe_obj_t *self = self_in;

    mp_printf(print, "CANFrame(CANID(id=");
    if (self->can_id & (1U << 29)) {
        mp_printf(print, "E%08"PRIx32"", self->can_id & 0x1fffffffU);
    }
    else {
        mp_printf(print, "S%03"PRIx32"", (self->can_id >> 18) & 0x7ffU);
    }

    mp_printf(print, "), dlc=%d, data=", self->dlc);

    if(self->remote) {
        mp_printf(print, "R");
    }
    else {
        uint32_t len = self->dlc > 8U ? 8U : self->dlc;
        if(self->dlc) {
            for (uint32_t i = 0; i < len; i++) {
                mp_printf(print, "%02x", ((uint8_t *) self->payload)[i]);
            }
        }
        else {
            mp_printf(print, "*");
        }
    }

    if (self->timestamp_valid) {
        mp_printf(print, ", timestamp=%lu", self->timestamp);
    }

    mp_printf(print, ")");
}

STATIC const mp_map_elem_t rp2_canframe_locals_dict_table[] = {
    // Instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_is_remote), (mp_obj_t)&rp2_canframe_is_remote_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_canid), (mp_obj_t)&rp2_canframe_get_canid_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_arbitration_id), (mp_obj_t)&rp2_canframe_get_arbitration_id_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_is_extended), (mp_obj_t)&rp2_canframe_is_extended_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_data), (mp_obj_t)&rp2_canframe_get_data_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_dlc), (mp_obj_t)&rp2_canframe_get_dlc_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_tag), (mp_obj_t)&rp2_canframe_get_tag_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_timestamp), (mp_obj_t)&rp2_canframe_get_timestamp_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_index), (mp_obj_t)&rp2_canframe_get_index_obj },
    // Static methods
    { MP_ROM_QSTR(MP_QSTR_from_bytes), (mp_obj_t)(&rp2_canframe_from_bytes_obj) },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canframe_locals_dict, rp2_canframe_locals_dict_table);

const mp_obj_type_t rp2_canframe_type = {
    { &mp_type_type },
    .name = MP_QSTR_CANFrame,
    .print = rp2_canframe_print,
    .make_new = rp2_canframe_make_new,
    .locals_dict = (mp_obj_t)&rp2_canframe_locals_dict,
};
///////////////////////////////////// End of CANFrame class //////////////////////////////////////

////////////////////////////////////// Start of CANID class //////////////////////////////////////
// Create the CANID instance and initialize the controller
STATIC mp_obj_t rp2_canid_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_arbitration_id,       MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0x7ffU}},
        {MP_QSTR_extended,             MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t arbitration_id = args[0].u_int;
    bool extended = args[1].u_bool;

    if (extended) {
        if ((arbitration_id < 0) || (arbitration_id >= (1U << 29))) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Extended arbitration ID values in range 0..0x1fffffff"));
        }
    }
    else {
        if ((arbitration_id < 0) || (arbitration_id >= (1U << 11))) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Standard arbitration ID values in range 0..0x7ff"));
        }
    }

    rp2_canid_obj_t *self = m_new_obj(rp2_canid_obj_t);
    self->base.type = &rp2_canid_type;

    self->can_id = extended ? arbitration_id | (1U << 29) : arbitration_id << 18;

    return self;
}

// Get a numeric value for the CAN ID, with 11-bit IDs in the range 0..7ff
STATIC mp_obj_t rp2_canid_get_arbitration_id(mp_obj_t self_in)
{
    rp2_canid_obj_t *self = self_in;

    uint32_t arbitration_id = self->can_id & (1U << 29) ? self->can_id & 0x1fffffffU : (self->can_id >> 18) & 0x7ffU;

    return MP_OBJ_NEW_SMALL_INT(arbitration_id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canid_get_arbitration_id_obj, rp2_canid_get_arbitration_id);

// Returns True if the ID is an extended ID, False otherwise
STATIC mp_obj_t rp2_canid_is_extended(mp_obj_t self_in)
{
    rp2_canid_obj_t *self = self_in;

    return self->can_id & (1U << 29) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canid_is_extended_obj, rp2_canid_is_extended);

// Get a CANID instance that will filter this ID
STATIC mp_obj_t rp2_canid_get_id_filter(mp_obj_t self_in)
{
    rp2_canid_obj_t *self = self_in;
    rp2_canidfilter_obj_t *filter = m_new_obj(rp2_canidfilter_obj_t);
    filter->base.type = &rp2_canidfilter_type;

    uint32_t eid = self->can_id & 0x3ffffU;
    uint32_t sid = (self->can_id >> 18 )& 0x7ffU;

    if (self->can_id & (1U << 29)) {
        filter->fltobj = (1U << 30) | (eid << 11) | sid;
        filter->mask = 0x5fffffffU;
    }
    else {
        filter->fltobj = sid;
        filter->mask = 0x400007ffU;
    }

    return filter;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canid_get_id_filter_obj, rp2_canid_get_id_filter);

STATIC void rp2_canid_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canid_obj_t *self = self_in;

    if (self->can_id & (1U << 29)) {
        mp_printf(print, "CANID(id=E%03"PRIx32")", self->can_id & 0x1fffffffU);
    }
    else {
        mp_printf(print, "CANID(id=S%03"PRIx32")", (self->can_id >> 18) & 0x7ffU);
    }
}

STATIC const mp_map_elem_t rp2_canid_locals_dict_table[] = {
    // Instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_arbitration_id), (mp_obj_t)&rp2_canid_get_arbitration_id_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_is_extended), (mp_obj_t)&rp2_canid_is_extended_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_id_filter), (mp_obj_t)&rp2_canid_get_id_filter_obj },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canid_locals_dict, rp2_canid_locals_dict_table);

const mp_obj_type_t rp2_canid_type = {
    { &mp_type_type },
    .name = MP_QSTR_CANID,
    .print = rp2_canid_print,
    .make_new = rp2_canid_make_new,
    .locals_dict = (mp_obj_t)&rp2_canid_locals_dict,
};
////////////////////////////////////// End of CANID class //////////////////////////////////////

///////////////////////////////// Start of CANIDFilter class ///////////////////////////////////
// Create the CANID instance and initialize the controller
STATIC mp_obj_t rp2_canidfilter_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_filter,     MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t mask_obj = args[0].u_obj;

    rp2_canidfilter_obj_t *self = m_new_obj(rp2_canidfilter_obj_t);
    self->base.type = &rp2_canidfilter_type;

    uint32_t mask = 0;
    uint32_t match = 0;

    if (mask_obj == MP_OBJ_NULL || mask_obj == mp_const_none) {
        // Set up a filter for "accept all"
        self->mask = 0;
        self->fltobj = 0;
    }
    else {
        if (!MP_OBJ_IS_STR_OR_BYTES(mask_obj)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Filter must be a String or bytes"));
        }

        mp_buffer_info_t bufinfo;
        uint8_t data[1];
        rp2_buf_get_for_send(mask_obj, &bufinfo, data);

        if (bufinfo.len != 29U && bufinfo.len != 11U) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Filter must be 11 or 29 characters long"));
        }

        bool extended = bufinfo.len == 29U;

        for (mp_uint_t i = 0; i < bufinfo.len; i++) {
            char ch = (((byte *) bufinfo.buf)[i]);

            if (ch == '1') {
                mask = (mask  << 1) | 1U;
                match = (match  << 1) | 1U;
            }
            else if (ch == '0') {
                mask = (mask << 1) | 1U;
                match = (match  << 1);
            }
            else if (ch == 'X') {
                mask <<= 1;
                match <<= 1;
            }
            else {
                nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Illegal character in filter: must be '1', '0' or 'X'"));
            }
        }

        if (extended) {
            // Need to break these into A and B for extended IDs because of the odd way that the MCP2517/18FD lays
            // out its filter registers
            uint32_t match_id_a = (match >> 18) & 0x7ffU;
            uint32_t match_id_b = match & 0x3ffffU;

            uint32_t mask_id_a = (mask >> 18) & 0x7ffU;
            uint32_t mask_id_b = mask & 0x3ffffU;

            self->mask = (1U << 30) | (mask_id_b << 11) | mask_id_a;            // Must match IDE value
            self->fltobj = (1U << 30) | (match_id_b << 11) | match_id_a;        // .. of IDE=1
        }
        else {
            uint32_t match_id_a = match & 0x7ffU;
            uint32_t mask_id_a = mask & 0x7ffU;

            self->mask = (1U << 30) | mask_id_a;                                // Must match IDE value
            self->fltobj = match_id_a;                                          // .. of IDE=0
        }
    }

    return self;
}

// Set the filter directly for testing purposes
STATIC mp_obj_t rp2_canidfilter_test_set_filter(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            {MP_QSTR_fltobj,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
            {MP_QSTR_mask,       MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
    };

    rp2_canidfilter_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t fltobj_obj = args[0].u_obj;
    mp_obj_t mask_obj = args[1].u_obj;

    if (!MP_OBJ_IS_INT(fltobj_obj)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "fltobj must be an integer"));
    }
    if (!MP_OBJ_IS_INT(mask_obj)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "mask must be an integer"));
    }
    uint32_t fltobj = mp_obj_get_int(fltobj_obj);
    uint32_t mask = mp_obj_get_int(mask_obj);

    self->fltobj = fltobj;
    self->mask = mask;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canidfilter_test_set_filter_obj, 1, rp2_canidfilter_test_set_filter);

STATIC void rp2_canidfiler_print_mask(const mp_print_t *print, bool id_a, uint32_t mask, uint32_t fltobj)
{
    if (id_a) {
        for(uint32_t i = 0; i < 11U; i++) {
            if (mask & (1U << (10U - i))) {
                // Must match
                if (fltobj & (1U << (10U - i))) {
                    mp_printf(print, "1");
                }
                else {
                    mp_printf(print, "0");
                }
            }
            else {
                // Don't care
                mp_printf(print, "X");
            }
        }
    }
    else {
        for(uint32_t i = 0; i < 18U; i++) {
            if (mask & (1U << (28U - i))) {
                // Must match
                if (fltobj & (1U << (28U - i))) {
                    mp_printf(print, "1");
                }
                else {
                    mp_printf(print, "0");
                }
            }
            else {
                // Don't care
                mp_printf(print, "X");
            }
        }
    }
}

STATIC void rp2_canidfilter_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canidfilter_obj_t *self = self_in;

    mp_printf(print, "CANIDFilter(filter=");

    // TODO print the 11/29 bit mask, with "Accept all" if 0/0
    if (self->mask == 0 && self->fltobj == 0) {
        mp_printf(print, "*");
    }
    else {
        if (self->fltobj & (1U << 30)) {
            // ID A
            rp2_canidfiler_print_mask(print, true, self->mask, self->fltobj);
            // ID B
            rp2_canidfiler_print_mask(print, false, self->mask, self->fltobj);
        }
        else {
            // ID A
            rp2_canidfiler_print_mask(print, true, self->mask, self->fltobj);
        }
    }

    mp_printf(print, ")");
}

STATIC const mp_map_elem_t rp2_canidfilter_locals_dict_table[] = {
        // Instance methods
        { MP_OBJ_NEW_QSTR(MP_QSTR_test_set_filter), (mp_obj_t)&rp2_canidfilter_test_set_filter_obj },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canidfilter_locals_dict, rp2_canidfilter_locals_dict_table);

const mp_obj_type_t rp2_canidfilter_type = {
        { &mp_type_type },
        .name = MP_QSTR_canidfilter,
        .print = rp2_canidfilter_print,
        .make_new = rp2_canidfilter_make_new,
        .locals_dict = (mp_obj_t)&rp2_canidfilter_locals_dict,
};

////////////////////////////////////// End of CANIDFilter class //////////////////////////////////////

////////////////////////////////////// Start of CANError class //////////////////////////////////////
// Create the CANError instance and initialize the controller
STATIC mp_obj_t rp2_canerror_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
            {MP_QSTR_c1bdiag1,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t c1bdiag1 = args[0].u_int;

    rp2_canerror_obj_t *self = m_new_obj(rp2_canerror_obj_t);
    self->base.type = &rp2_canerror_type;
    self->c1bdiag1 = c1bdiag1;
    self->timestamp = 0;

    return self;
}

// Returns the error frame's timestamp, or None if no timestamp valid
STATIC mp_obj_t rp2_canerror_get_timestamp(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return mp_obj_new_int_from_uint(self->timestamp);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_get_timestamp_obj, rp2_canerror_get_timestamp);

STATIC mp_obj_t rp2_canerror_is_crc_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 21) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_crc_error_obj, rp2_canerror_is_crc_error);

STATIC mp_obj_t rp2_canerror_is_stuff_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 20) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_stuff_error_obj, rp2_canerror_is_stuff_error);

STATIC mp_obj_t rp2_canerror_is_form_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 19) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_form_error_obj, rp2_canerror_is_form_error);

STATIC mp_obj_t rp2_canerror_is_ack_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 18) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_ack_error_obj, rp2_canerror_is_ack_error);

STATIC mp_obj_t rp2_canerror_is_bit1_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 17) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_bit1_error_obj, rp2_canerror_is_bit1_error);

STATIC mp_obj_t rp2_canerror_is_bit0_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 16) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_bit0_error_obj, rp2_canerror_is_bit0_error);

STATIC mp_obj_t rp2_canerror_is_bus_off(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;

    return self->c1bdiag1 & (1U << 23) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_bus_off_obj, rp2_canerror_is_bus_off);


STATIC void rp2_canerror_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canerror_obj_t *self = self_in;

    mp_printf(print, "CANError(");
    bool prev_item = false;
    // Prints the error type
    if (self->c1bdiag1 & (1U << 23)) {
        mp_printf(print, "bus_off=True");
        prev_item = true;
    }
    if (self->c1bdiag1 & (1U << 21)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "crc_error=True");
        prev_item = true;
    }
    if (self->c1bdiag1 & (1U << 20)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "stuff_error=True");
        prev_item = true;
    }
    if (self->c1bdiag1 & (1U << 19)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "form_error=True");
        prev_item = true;
    }
    if (self->c1bdiag1 & (1U << 18)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "ack_error=True");
        prev_item = true;
    }
    if (self->c1bdiag1 & (1U << 17)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "bit1_error=True");
        prev_item = true;
    }
    if (self->c1bdiag1 & (1U << 16)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "bit0_error=True");
    }
    if (prev_item) {
        mp_printf(print, ", ");
    }
    mp_printf(print, "frame_cnt=%d, timestamp=%lu)", self->c1bdiag1 & 0xffffU, self->timestamp);
}

STATIC const mp_map_elem_t rp2_canerror_locals_dict_table[] = {
        // Instance methods
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_timestamp), (mp_obj_t)&rp2_canerror_get_timestamp_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_crc_error), (mp_obj_t)&rp2_canerror_is_crc_error_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_stuff_error), (mp_obj_t)&rp2_canerror_is_stuff_error_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_form_error), (mp_obj_t)&rp2_canerror_is_form_error_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_ack_error), (mp_obj_t)&rp2_canerror_is_ack_error_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_bit1_error), (mp_obj_t)&rp2_canerror_is_bit1_error_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_bit0_error), (mp_obj_t)&rp2_canerror_is_bit0_error_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_is_bus_off), (mp_obj_t)&rp2_canerror_is_bus_off_obj },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canerror_locals_dict, rp2_canerror_locals_dict_table);

const mp_obj_type_t rp2_canerror_type = {
        { &mp_type_type },
        .name = MP_QSTR_CANError,
        .print = rp2_canerror_print,
        .make_new = rp2_canerror_make_new,
        .locals_dict = (mp_obj_t)&rp2_canerror_locals_dict,
};
////////////////////////////////////// End of CANError class //////////////////////////////////////
