// Copyright 2020 Canis Automotive Labs (https://canislabs.com)
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

#ifndef MICROPYTHON_CAN_H
#define MICROPYTHON_CAN_H

#include <py/obj.h>

#include "rp2_mcp251718fd.h"

#define MCP251718FD_TX_QUEUE_SIZE               (32U)           // Must be less <= 32
#define MCP251718FD_TX_FIFO_SIZE                (32U)           // Must be less <= 32
#define RX_FIFO_SIZE                            (128U)          // Must be less than 256
#define TX_EVENT_FIFO_SIZE                      (128U)          // Must be less than 256

#define NUM_FRAME_BYTES                         (19U)
#define NUM_EVENT_BYTES                         (9U)

#ifdef CAN_DEBUG
#define CAN_DEBUG_PRINT(fmt, args...)   printf(fmt, ##args)
#else
#define CAN_DEBUG_PRINT(fmt, args...)   /* */
#endif

void can_init(void);
void can_deinit(void);

/////////////// The binary version of a received CAN frame as bytes is laid out as follows:
//
// Byte 0: Flags:
//      bits 3:0 = event type (0 = transmitted frame, 1 = received frame, 2 = overflow event record, 3 = CAN error)
//      bits 6:4 = reserved (must be set to 0)
//      bit 7    = remote frame
// Bytes 1-4: timestamp (received) or tag (transmitted) (Big endian)
//
// Frame                                        Overflow                                    Error
// -----                                        --------                                    -----
// Byte 5: DLC
// Byte 6: Filter index
// Bytes 7-10:  CAN ID in 32-bit                Bytes 7-10: Frame overflow count            Bytes 7-10: Error info
//              format (Big endian)
// Bytes 11-18: Payload (padded to 8 bytes)     Bytes 11-14: Error overflow count
//
//
/////////////// Binary version of a CAN frame to transmit as bytes is laid out as follows:
//
// Byte 0:      Flags, as follows:
//      bits 6:0 = reserved (must be set to 0)
//      bit 7    = remote
// Bytes 1-4:   Tag (Big endian)
// Byte 5:      DLC
// Byte 6:      Filter index
// Bytes 7-10:  CAN ID in 32-bit format (Big endian)
// Bytes 11-18: Payload (padded to8 bytes)
//
// CAN ID is a 32-bit integer laid out as follows:
//
//      31       23       15       7
//      V        V        V        V
//      ++++++++ ++++++++ ++++++++ ++++++++
//      --EAAAAA AAAAAABB BBBBBBBB BBBBBBBB
//
//      E       = Extended ID
//      A       = 11-bit ID A
//      B       = 18-bit ID B
//
//////////////// Binary version of a CAN transmit event is laid out as follows:
//
// Byte 0:      Flags, as follows:
//      bits 3:0 = event type (0 = transmitted frame, 2 = overflow event record)
// Bytes 1-4: tag (in big endian format)
// Bytes 5-8: timestamp (in big endian format)
//
//////////////// Binary version of a trigger is laid out as follows:
//
// Byte 0:      Flags, as follows:
//      bits 6:0    = reserved (must be 0)
//      bits 7      = on error
// Bytes 1-4:   canid mask (in big endian format)
// Bytes 5-8:   canid match
// Byte 9:      DLC mask
// Byte 10:     DLC match
// Bytes 11-18: Payload mask
// Bytes 19-26: Payload match



typedef uint32_t canid_t;

typedef enum {
    CAN_BITRATE_500K_75,
    CAN_BITRATE_250K_75,
    CAN_BITRATE_125K_75,
    CAN_BITRATE_1M_75,
    CAN_BITRATE_500K_50,
    CAN_BITRATE_250K_50,
    CAN_BITRATE_125K_50,
    CAN_BITRATE_1M_50,
    CAN_BITRATE_2M_50,
    CAN_BITRATE_4M_90,
    CAN_BITRATE_2_5M_75 ,
    CAN_BITRATE_2M_80,
} canprofile_t;

typedef enum {
    EVENT_TYPE_TRANSMITTED_FRAME = 0,
    EVENT_TYPE_RECEIVED_FRAME,
    EVENT_TYPE_OVERFLOW,
    EVENT_TYPE_CAN_ERROR
} canevent_type_t;

typedef enum {
    CAN_MODE_NORMAL,
    CAN_MODE_LISTEN_ONLY,
    CAN_MODE_ACK_ONLY,
    CAN_MODE_OFFLINE
} canmode_t;

typedef struct  {
    canid_t canid;
    uint32_t payload[2];
    uint8_t dlc;
    uint8_t id_filter;
    bool remote;
} rx_canframe_t;

typedef struct  {
    uint32_t frame_cnt;
    uint32_t error_cnt;
} rx_overflow_cnt_t;


typedef struct  {
    union {
        rx_canframe_t rx_frame;
        rx_overflow_cnt_t overflow_cnt;
        uint32_t c1bdiag1;
    } info;
    uint32_t timestamp;                                 // SOF, microseconds (always valid)
    canevent_type_t event_type;
} rx_event_t;

typedef struct  {
    union {
        uint32_t generic;                               // Placeholder for all of the words
        uint32_t frame_tag;                             // Supplied via API
        uint32_t overflow_cnt;                          // Counts transmit overflow events
        uint32_t error_info;                            // Details about the error
    } info;
    uint32_t timestamp;                                 // SOF, microseconds
    canevent_type_t event_type;                         // Indicates if a frame transmit, error or overflow
} tx_event_t;

typedef struct {
    mp_obj_base_t base;
    uint32_t tag;                                       // Tag supplied via API for identification of an instance
    canid_t can_id;
    uint32_t payload[2];
    uint32_t timestamp;                                 // SOF, microseconds
    uint8_t dlc;
    uint8_t id_filter;
    bool remote;
    bool timestamp_valid;                               // true when timestamp is set; cleared when queued for transmission
} rp2_canframe_obj_t;

// Software receive FIFO
// This stores details about the CAN frame necessary to create an instance of CANFrame when the
// recv() call is made
typedef struct {
    rx_event_t rx_events[RX_FIFO_SIZE];
    uint8_t head_idx;                                   // Index into first used frame in the FIFO
    uint8_t tail_idx;                                   // Index into first free frame at the back of the FIFO
    uint8_t free;                                       // Number of free slots in the queue
    uint8_t dropped_event_idx;                          // Used when the FIFO has overrun
} can_rx_fifo_t;

// Software transmit FIFO; the CAN frame already exists in the MicroPython heap so a
// reference to it is used
typedef struct {
    rp2_canframe_obj_t *frames[MCP251718FD_TX_FIFO_SIZE]; // Frames created on the heap
    uint8_t head_idx;                                   // Index into first used frame in the FIFO
    uint8_t tail_idx;                                   // Index into first free frame at the back of the FIFO
    uint8_t num_free_slots;                             // Number of free slots in the queue
} can_tx_fifo_t;

// Software shadow structure for the transmit queue
//
// When a frame is queued, a pointer to it is put into this array and the sequence number assigned to
// the index. That index is used later on transmission to take it out and fill in the timestamp.
typedef struct {
    rp2_canframe_obj_t *frames[MCP251718FD_TX_QUEUE_SIZE]; // The frames (directly inside MicroPython objects) queued
    uint32_t num_free_slots;                            // Number of free slots in the queue
    uint8_t fifo_slot;                                  // If fifo_slot is < MCP251718FD_TX_QUEUE_SIZE true then indicates where the FIFO frame is
} can_tx_queue_t;

// Transmit event FIFO that records the timestamp and the tag of the transmitted frame
typedef struct {
    tx_event_t events[TX_EVENT_FIFO_SIZE];
    uint8_t head_idx;                                   // Index into first used frame in the FIFO
    uint8_t tail_idx;                                   // Index into first free frame at the back of the FIFO
    uint8_t free;                                       // Number of free slots in the queue
    uint8_t dropped_event_idx;
} can_event_fifo_t;

// Transmit event FIFO that records the timestamp and the tag of the transmitted frame
typedef struct {
    uint32_t can_id_mask;                               // Mask/match values over ID, DLC and payload
    uint32_t can_id_match;
    uint32_t can_payload_mask[2];
    uint32_t can_payload_match[2];
    uint8_t can_dlc_mask;
    uint8_t can_dlc_match;
    bool on_error;                                      // Set if should trigger on error
    bool on_rx;                                         // Set if should trigger on receiving a matching frame
    bool enabled;                                       // Set if trigger is enabled
} can_trigger_t;

// The four CAN API classes:
//
// CAN
// CANID
// CANFrame
// CANIDFilter

typedef struct _rp2_canid_obj_t {
    mp_obj_base_t base;
    canid_t can_id;
} rp2_canid_obj_t;

typedef struct _rp2_canidfilter_obj_t {
    mp_obj_base_t base;
    uint32_t mask;
    uint32_t fltobj;
} rp2_canidfilter_obj_t;

typedef struct _rp2_can_obj_t {
    mp_obj_base_t base;
    can_rx_fifo_t rx_fifo;
    can_tx_fifo_t tx_fifo;
    can_tx_queue_t tx_queue;
    can_event_fifo_t event_fifo;
    can_trigger_t triggers[1];                          // Only one trigger at present
    canmode_t mode;
    uint32_t seq_bad;                                   // Number of times the SEQ field was bad
    uint32_t txqua_bad;                                 // C1TXQUA read back out of range or otherwise bad
    uint32_t txqsta_bad;                                // C1TXSTA read back showing queue full
    bool recv_errors;                                   // Whether to record errors in the receive FIFO
} rp2_can_obj_t;

typedef struct _rp2_canerror_obj_t {
    mp_obj_base_t base;
    uint32_t c1bdiag1;                                  // Value of error register
    uint32_t timestamp;                                 // Read from the controller
} rp2_canerror_obj_t;

extern const mp_obj_type_t rp2_can_type;
extern const mp_obj_type_t rp2_canid_type;
extern const mp_obj_type_t rp2_canframe_type;
extern const mp_obj_type_t rp2_canidfilter_type;
extern const mp_obj_type_t rp2_canerror_type;

#endif // MICROPYTHON_CAN_H
