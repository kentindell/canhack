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

#include "py/obj.h"

#include "canapi.h"

#ifdef CAN_DEBUG
#define CAN_DEBUG_PRINT(fmt, args...)   mp_printf(MP_PYTHON_PRINTER, fmt, ##args)
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
// Bytes 11-18: Data (padded to 8 bytes)        Bytes 11-14: Error overflow count
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
// Bytes 11-18: Data (padded to 8 bytes)
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
// Bytes 11-18: Data mask
// Bytes 19-26: Data match

// Transmit event FIFO that records the timestamp and the tag of the transmitted frame
typedef struct {
    uint32_t arbitration_id_mask;                       // Mask/match values over ID, DLC and data
    uint32_t arbitration_id_match;
    bool ide_match;
    uint32_t can_data_mask[2];
    uint32_t can_data_match[2];
    uint8_t can_dlc_mask;
    uint8_t can_dlc_match;
    bool on_error;                                      // Set if should trigger on error
    bool on_rx;                                         // Set if should trigger on receiving a matching frame
    bool on_tx;                                         // Set if should trigger on a transmitting a matching frame
    bool enabled;                                       // Set if trigger is enabled
} can_trigger_t;

// The six CAN API classes:
//
// CAN
// CANID
// CANFrame
// CANIDFilter
// CANError
// CANOverflow

typedef struct _rp2_can_obj_t {
    mp_obj_base_t base;
    can_controller_t controller;
    can_trigger_t triggers[1];                          // TODO allow multiple triggers
    mp_obj_t mp_rx_callback_fn;                         // Python function to call on receive
} rp2_can_obj_t;

typedef struct _rp2_canid_obj_t {
    mp_obj_base_t base;
    uint32_t arbitration_id;                            // 11- or 29-bit arbitration ID
    bool extended;                                      // IDE bit
} rp2_canid_obj_t;

typedef struct {
    mp_obj_base_t base;
    can_frame_t frame;
    uint32_t tag;                                       // Tag supplied via API for identification of an instance
    uint32_t timestamp;
    bool timestamp_valid;                               // true when timestamp is set; cleared when queued for transmission
} rp2_canframe_obj_t;

typedef struct _rp2_canidfilter_obj_t {
    mp_obj_base_t base;
    can_id_filter_t filter;
} rp2_canidfilter_obj_t;

typedef struct _rp2_canerror_obj_t {
    mp_obj_base_t base;
    can_error_t error;                                  // Value of error register
    uint32_t timestamp;                                 // Read from the controller
} rp2_canerror_obj_t;

typedef struct _rp2_canoverflow_obj_t {
    mp_obj_base_t base;
    uint32_t error_cnt;
    uint32_t frame_cnt;    
    bool receive;                                       // Receive overflow
    uint32_t timestamp;                                 // Timestamp associated with overflow
} rp2_canoverflow_obj_t;

extern const mp_obj_type_t rp2_can_type;
extern const mp_obj_type_t rp2_canid_type;
extern const mp_obj_type_t rp2_canframe_type;
extern const mp_obj_type_t rp2_canidfilter_type;
extern const mp_obj_type_t rp2_canerror_type;
extern const mp_obj_type_t rp2_canoverflow_type;

#endif // MICROPYTHON_CAN_H
