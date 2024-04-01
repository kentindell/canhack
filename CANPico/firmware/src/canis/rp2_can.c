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
// TODO more than TRIG pin 1 trigger with an OR condition between them

#define TRIG_SET()                          (sio_hw->gpio_set = (1U << TRIG_GPIO))
#define TRIG_CLEAR()                        (sio_hw->gpio_clr = (1U << TRIG_GPIO))
#define NOP()                               __asm__("nop");

#define TRIG_GPIO                           (2U)

#define FRAME_FROM_BYTES_NUM                (19U)

#ifdef NOTDEF
// Only used for debugging to print from outside MicroPython firmware
void debug_printf( const char *format, ... )
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    mp_printf(MP_PYTHON_PRINTER, "%s", buffer);
    va_end(args);
}
#endif

STATIC void TIME_CRITICAL irq_handler(void)
{
    // Get a pointer to the CAN controller
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj[0]);
    can_controller_t *controller = &self->controller;

    // Work out if this interrupt is from the the MCP25xxFD. The bound interface
    // defines the pin used for the interrupt line from the CAN controller.
    uint8_t spi_irq = controller->host_interface.spi_irq;
    uint32_t events = gpio_get_irq_event_mask(spi_irq); 

    if (events & GPIO_IRQ_LEVEL_LOW) {
        mcp25xxfd_irq_handler(controller);
    }
}

// In the future there may be multiple CAN controllers on a CANPico board and
// so they will all be initialized/de-initialized here.
void can_init(void) {
    // Set up the root pointer to a null CAN controller object so that the memory is not allocate until CAN is used.
    MP_STATE_PORT(rp2_can_obj[0]) = MP_OBJ_NULL;
}

void can_deinit(void) {
    // Called when the system is soft reset (CTRL-D in REPL).

    // If the controller is initialized then take it offline and deactivate it
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj[0]);
    if (self != MP_OBJ_NULL) {
        can_stop_controller(&self->controller);
        irq_remove_handler(IO_IRQ_BANK0, irq_handler);
    }
    MP_STATE_PORT(rp2_can_obj[0]) = MP_OBJ_NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Start of MicroPython bindings //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////// Start of CAN class //////////////////////////////////////

// Create the CAN instance and initialize the controller
STATIC mp_obj_t rp2_can_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_profile,           MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 0}},
        {MP_QSTR_id_filters,        MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
        {MP_QSTR_hard_reset,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},
        {MP_QSTR_brp,               MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = -1}},
        {MP_QSTR_tseg1,             MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 10}},
        {MP_QSTR_tseg2,             MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 3}},
        {MP_QSTR_sjw,               MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int  = 2}},
        {MP_QSTR_recv_errors,       MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},
        {MP_QSTR_mode,              MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_tx_open_drain,     MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = true}},
        {MP_QSTR_reject_remote,     MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},
        {MP_QSTR_rx_callback_fn,    MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}}, 
        {MP_QSTR_recv_overflows,    MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false}},               
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t profile = args[0].u_int;
    mp_obj_dict_t *mp_id_filters = args[1].u_obj;
    bool hard_reset = args[2].u_bool;
    int brp = args[3].u_int;
    u_int tseg1 = args[4].u_int;
    u_int tseg2 = args[5].u_int;
    u_int sjw = args[6].u_int;
    bool recv_errors = args[7].u_bool;
    can_mode_t mode = args[8].u_int;
    bool tx_open_drain = args[9].u_bool;
    bool reject_remote = args[10].u_bool;
    mp_obj_t mp_rx_callback_fn = args[11].u_obj;
    bool recv_overflows = args[12].u_bool;

    can_bitrate_t bitrate = {.profile=profile,
                             .brp=brp,
                             .tseg1=tseg1,
                             .tseg2=tseg2,
                             .sjw=sjw};

    // Modes are:
    // 0: (default) CAN.NORMAL, start normally
    // 1: CAN.LISTEN_ONLY, does not ever set TX to 0
    // 2: CAN.ACK_ONLY, does not transmit but does set ACK=0
    // 3: CAN.OFFLINE, does not send or receive

    if (mp_rx_callback_fn != mp_const_none && !MP_OBJ_IS_FUN(mp_rx_callback_fn)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "rx_callback_fn must be a function"));
    }

    if (mp_id_filters != mp_const_none) {
        // Check dictionary is well-formed
        if(!MP_OBJ_IS_TYPE(mp_id_filters, &mp_type_dict)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "A dict expected for id_filters"));
        }

        for(uint32_t idx = 0; idx < CAN_MAX_ID_FILTERS; idx++) {
            mp_map_elem_t *elem = mp_map_lookup(&mp_id_filters->map, MP_OBJ_NEW_SMALL_INT(idx), MP_MAP_LOOKUP);
            if (elem != NULL) {
                if(!MP_OBJ_IS_TYPE(elem->value, &rp2_canidfilter_type)) {
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "A CANIDFilter instance expected for filter %d", idx));
                }
            }
        }
    }

    // Create class instance for controller
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj[0]);

    if (self == MP_OBJ_NULL) {
        // Newly create object (we don't want it created always because it's a fairly large object, with
        // large receive FIFO and this shouldn't be allocated until needed).
        self = m_new_obj(rp2_can_obj_t);
        self->base.type = &rp2_can_type;
        MP_STATE_PORT(rp2_can_obj[0]) = self;
        // Bind the interrupt handler from the GPIO port
        irq_add_shared_handler(IO_IRQ_BANK0, irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    }

    // Up to 32 filters can be set
    can_id_filter_t filters[CAN_MAX_ID_FILTERS];

    // Add in the filters
    if (mp_id_filters != mp_const_none) {
        CAN_DEBUG_PRINT("Setting specific filters\n");
        for (uint32_t idx = 0; idx < CAN_MAX_ID_FILTERS; idx++) {
            mp_map_elem_t *elem = mp_map_lookup(&mp_id_filters->map, MP_OBJ_NEW_SMALL_INT(idx), MP_MAP_LOOKUP);
            if (elem != NULL) {
                CAN_DEBUG_PRINT("Filter index=%d\n", idx);
                rp2_canidfilter_obj_t *mp_filter = elem->value;
                // Filter has already been made
                filters[idx] = mp_filter->filter;
            }
            else {
                // Not defined, set a disabled filter
                can_make_id_filter_disabled(&filters[idx]);
            }
        }
    }

    can_id_filters_t all_filters = {.filter_list = filters, .n_filters = CAN_MAX_ID_FILTERS};

    uint16_t options = 0;
    if (recv_errors) {
        options |= CAN_OPTION_RECV_ERRORS;
    }
    if (reject_remote) {
        options |= CAN_OPTION_REJECT_REMOTE;
    }
    if (tx_open_drain) {
        options |= CAN_OPTION_OPEN_DRAIN;
    }
    if (hard_reset) {
        options |= CAN_OPTION_HARD_RESET;
    }
    if (!recv_overflows) {
        options |= CAN_OPTION_REJECT_OVERFLOW;
    }
    options |= CAN_OPTION_RECORD_TX_EVENTS;

    // Bind the host interface to the CANPico's pin layout
    mcp25xxfd_spi_bind_canpico(&self->controller.host_interface);
    // Can now call the initialize with the interface bound
    can_errorcode_t rc = can_setup_controller(&self->controller, &bitrate, &all_filters, mode, options);
    if (rc == CAN_ERC_BAD_INIT) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "Hardware error: Cannot put CAN controller into config mode"));
    }
    if (rc == CAN_ERC_NO_INTERFACE) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "No SPI interface bound to controller"));
    }
    if (rc == CAN_ERC_BAD_WRITE) {
        // For the MCP25xxFD this is returned when the transmit open drain bit is not set after being requested
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "Failed to set tx_open_drain"));
    }
    if (rc != CAN_ERC_NO_ERROR) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "Unknown error code: %d", rc));
    }
    // Set the callback function that will be called with a received frame
    self->mp_rx_callback_fn = mp_rx_callback_fn;

    return self;
}

STATIC mp_obj_t rp2_can_send_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_frame,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},
        {MP_QSTR_fifo,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    can_controller_t *controller = &self->controller;

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canframe_obj_t *mp_frame = args[0].u_obj;
    bool fifo = args[1].u_bool;

    if(!MP_OBJ_IS_TYPE(mp_frame, &rp2_canframe_type)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "CANFrame expected"));
    }

    // C API call
    can_errorcode_t rc = can_send_frame(controller, &mp_frame->frame, fifo);

    if (rc == CAN_ERC_NO_ROOM_FIFO) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in FIFO queue"));
    }
    if (rc == CAN_ERC_NO_ROOM_PRIORITY) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in priority queue"));
    }
    if (rc == CAN_ERC_NO_ROOM) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "No room in transmit queue"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_send_frame_obj, 1, rp2_can_send_frame);

STATIC mp_obj_t rp2_can_send_frames(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_frames,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},
        {MP_QSTR_fifo,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    can_controller_t *controller = &self->controller;
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_list_t *frames = args[0].u_obj;
    bool fifo = args[1].u_bool;

    if(!MP_OBJ_IS_TYPE(frames, &mp_type_list)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "List of CAN frames expected"));
    }
    // Quick check to see that the list contains only CAN frames
    for (uint32_t i = 0; i < frames->len; i++) {
        rp2_canframe_obj_t *mp_frame = frames->items[i];
        if(!MP_OBJ_IS_TYPE(mp_frame, &rp2_canframe_type)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "CANFrame expected"));
        }
    }

    if (can_is_space(controller, frames->len, fifo)) {
        for (uint32_t i = 0; i < frames->len; i++) {
            rp2_canframe_obj_t *mp_frame = frames->items[i];
            can_send_frame(controller, &mp_frame->frame, fifo);
        }
    }
    else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "No space to transmit all frames"));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_send_frames_obj, 1, rp2_can_send_frames);

STATIC mp_obj_t rp2_can_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_limit,         MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CAN_RX_FIFO_SIZE}},
        {MP_QSTR_as_bytes,      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    can_controller_t *controller = &self->controller;
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // rx_fifo.free is safe to access outside the ISR because it's an atomic word and can only decrease so num_frames can only increase
        
    uint32_t limit = args[0].u_int;
    bool as_bytes = args[1].u_bool;

    uint32_t num_events = can_recv_pending(controller);
    if (limit > num_events) {
        limit = num_events;
    }

    // If the queue is empty then return a result that does not use the heap: otherwise spinning
    // on recv() will invoke the garbage collector.

    if (num_events == 0) {
        if (as_bytes) {
            return mp_const_empty_bytes;
        }
        else {
            return mp_const_empty_tuple;
        }
    }

    if (as_bytes) {
        // Temporary buffer to store some frames
        uint8_t buf[255];

        size_t remaining = sizeof(buf);
        size_t n = 0;

        // Pull frames from the FIFO up to a limit, keeping track of the bytes added so that if
        // there aren't enough bytes then that's the number returned
        for (uint32_t i = 0; i < limit; i++) {
            size_t added = can_recv_as_bytes(controller, buf + n, remaining);
            if (added > 0) {
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
            can_rx_event_t event;
            can_rx_event_t *ev = &event;
            if (can_recv(controller, ev)) {
                if (can_event_is_frame(ev)) {
                    rp2_canframe_obj_t *mp_frame = m_new_obj(rp2_canframe_obj_t);
                    mp_frame->base.type = &rp2_canframe_type;
                    mp_frame->frame = *can_event_get_frame(ev); // Make a copy (ev is temporary)
                    mp_frame->timestamp = can_event_get_timestamp(ev);
                    mp_frame->timestamp_valid = true;
                    list->items[i] = mp_frame;
                }
                else if (can_event_is_error(ev)) {
                    rp2_canerror_obj_t *mp_error = m_new_obj(rp2_canerror_obj_t);
                    mp_error->base.type = &rp2_canerror_type;
                    mp_error->error = *can_event_get_error(ev); // Make a copy (ev is temporary)
                    mp_error->timestamp = can_event_get_timestamp(ev);
                    list->items[i] = mp_error;
                }
                else if (can_event_is_overflow(ev)) {
                    rp2_canoverflow_obj_t *mp_overflow = m_new_obj(rp2_canoverflow_obj_t);
                    mp_overflow->base.type = &rp2_canoverflow_type;
                    mp_overflow->receive = true;
                    mp_overflow->error_cnt = can_rx_overflow_get_error_cnt(&ev->event.overflow);
                    mp_overflow->frame_cnt = can_rx_overflow_get_frame_cnt(&ev->event.overflow);
                    mp_overflow->timestamp = can_event_get_timestamp(ev);
                    list->items[i] = mp_overflow;
                }
                else {
                    // Unknown event type, should never happen, but we set the list item
                    // to none just in case
                    list->items[i] = mp_const_none;
                }
            }
        }
        return list;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_recv_obj, 1, rp2_can_recv);

#if _BullseyeCoverage
// TODO allocate this on the heap?
// TODO restrict the coverage to certain files only
char cov_memoryArea[4096];
long cov_memoryAreaSize = 4096;

// TODO make a specific Python API call for dumping data
// #if _BullseyeCoverage
    // cov_dumpData();
// #endif

#endif

// Return number of messages waiting in the RX FIFO.`
STATIC mp_obj_t rp2_can_recv_pending(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;
    can_controller_t *controller = &self->controller;

    return MP_OBJ_NEW_SMALL_INT(can_recv_pending(controller));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_recv_pending_obj, rp2_can_recv_pending);

STATIC mp_obj_t rp2_can_recv_tx_events(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            {MP_QSTR_limit,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CAN_TX_EVENT_FIFO_SIZE}},
            {MP_QSTR_as_bytes, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    rp2_can_obj_t *self = pos_args[0];
    can_controller_t *controller = &self->controller;
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t limit = args[0].u_int;
    bool as_bytes = args[1].u_bool;

    uint32_t num_events = can_recv_tx_events_pending(controller);

    if (limit > num_events) {
        limit = num_events;
    }

    if (as_bytes) {
        uint8_t buf[128];
        size_t remaining = sizeof(buf);
        size_t n = 0;

        // Pull events from the FIFO up to a limit
        for (uint32_t i = 0; i < limit; i++) {
            size_t added = can_recv_tx_event_as_bytes(controller, buf + n, remaining);
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
            can_tx_event_t event;
            can_tx_event_t *e = &event;
            
            bool recvd = can_recv_tx_event(controller, e);
            if (recvd) {
                if (can_tx_event_is_frame(e)) {
                    // Return a reference to the instance of the transmitted frame (that should not have been garbage collected
                    // because the reference to it is in the controller structure).
                    //
                    // Frame transmitted, return the CANFrame instance (application can then dig out the tag etc.)
                    // The transmit ISR callback will have already run and put the timestamp into the CANFrame
                    // instance so no need to fill it in here.
                    list->items[i] = (rp2_canframe_obj_t *)(can_tx_event_get_uref(e).ref);
                }
                else {
                    // It's an overflow event, so return an CANOverflow instance
                    rp2_canoverflow_obj_t *mp_overflow = m_new_obj(rp2_canoverflow_obj_t);
                    mp_overflow->base.type = &rp2_canoverflow_type;
                    mp_overflow->receive = false;
                    mp_overflow->frame_cnt = can_tx_event_get_overflow_cnt(e);
                    mp_overflow->timestamp = can_tx_event_get_timestamp(e);
                    list->items[i] = mp_overflow;                    
                }
            }
            else {
                break;
            }
        }
        return list;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_recv_tx_events_obj, 1, rp2_can_recv_tx_events);

// Return number of events waiting in the TX event FIFO.`
STATIC mp_obj_t rp2_can_recv_tx_events_pending(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;
    can_controller_t *controller = &self->controller;

    return MP_OBJ_NEW_SMALL_INT(can_recv_tx_events_pending(controller));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_recv_tx_events_pending_obj, rp2_can_recv_tx_events_pending);

// Return number of messages waiting in the RX FIFO.
STATIC mp_obj_t rp2_can_get_status(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;
    can_controller_t *controller = &self->controller;
    can_status_t status = can_get_status(controller);

    // Returns a tuple of:
    // bool: is Bus-off
    // bool: is Error Passive
    // int: TEC
    // int: REC
    mp_obj_tuple_t *tuple = mp_obj_new_tuple(4U, NULL);
    tuple->items[0] = can_status_is_bus_off(status) ? mp_const_true : mp_const_false;
    tuple->items[1] = can_status_is_error_passive(status) ? mp_const_true : mp_const_false;
    tuple->items[2] = MP_OBJ_NEW_SMALL_INT(can_status_get_tec(status));
    tuple->items[3] = MP_OBJ_NEW_SMALL_INT(can_status_get_rec(status));

    return tuple;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_status_obj, rp2_can_get_status);

// Return diagnostic values
STATIC mp_obj_t rp2_can_get_diagnostics(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;
    can_controller_t *controller = &self->controller;

    // Returns a tuple of:
    //
    // integer: number of times SEQ was corrupted
    // integer: number of times TXQUA was read as corrupted
    // integer: number of times TXQSTA was read as corrupted
    // integer: number of times Bus Off happened
    // integer: number of times a spurious interrupt happened
    // integer: number of times an SPI read had a bad CRC
    //
    // This is target-specific MCP25xxFD code
    mp_obj_tuple_t *tuple = mp_obj_new_tuple(6U, NULL);
    tuple->items[0] = MP_OBJ_NEW_SMALL_INT(controller->target_specific.seq_bad);
    tuple->items[1] = MP_OBJ_NEW_SMALL_INT(controller->target_specific.txqua_bad);
    tuple->items[2] = MP_OBJ_NEW_SMALL_INT(controller->target_specific.txqsta_bad);
    tuple->items[3] = MP_OBJ_NEW_SMALL_INT(controller->target_specific.bus_off);
    tuple->items[4] = MP_OBJ_NEW_SMALL_INT(controller->target_specific.spurious);
    tuple->items[5] = MP_OBJ_NEW_SMALL_INT(controller->target_specific.crc_bad);

    return tuple;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_diagnostics_obj, rp2_can_get_diagnostics);

// Return the timestamp counter (Typically used to convert timestamps to time-of-day)
STATIC mp_obj_t rp2_can_get_time(mp_obj_t self_in)
{
    rp2_can_obj_t *self = self_in;
    can_controller_t *controller = &self->controller;

    return mp_obj_new_int_from_uint(can_get_time(controller));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_get_time_obj, rp2_can_get_time);

// Return the timestamp counter resolution in ticks per second (the CAN drivers
// set the timer to tick at 1us)
STATIC mp_obj_t rp2_can_get_time_hz(mp_obj_t self_in)
{
    // Not used
    // rp2_can_obj_t *self = pos_args[0];
    // can_controller_t *controller = &self->controller;

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
    can_controller_t *controller = &self->controller;

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool fifo = args[0].u_bool;

    return MP_OBJ_NEW_SMALL_INT(can_get_send_space(controller, fifo));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_can_get_send_space_obj, 1, rp2_can_get_send_space);

// Set the conditions for triggering an edge on the trigger pin
STATIC mp_obj_t rp2_can_set_trigger(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            {MP_QSTR_on_error,      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
            {MP_QSTR_on_canid,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none}},   // A specific CAN ID
            {MP_QSTR_as_bytes,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none}},   // A block of bytes
            {MP_QSTR_on_tx,         MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
            {MP_QSTR_on_rx,         MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}},
    };

    rp2_can_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool on_error = args[0].u_bool;
    rp2_canid_obj_t *mp_on_canid = args[1].u_obj;
    mp_obj_t as_bytes = args[2].u_obj;
    bool on_tx = args[3].u_bool;
    bool on_rx = args[4].u_bool;

    if (as_bytes != mp_const_none) {
        // Trigger can be set directly but the ID trigger is then not valid
        if (!MP_OBJ_IS_STR_OR_BYTES(as_bytes)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Trigger must be bytes"));
        }
        if (mp_on_canid != mp_const_none) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Cannot set a binary trigger and an ID trigger"));
        }

        uint8_t trigger_buf[27];
        uint32_t len = (uint8_t)copy_mp_bytes(as_bytes, (uint8_t *)trigger_buf, sizeof(trigger_buf));
        if (len != sizeof(trigger_buf)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Trigger must be %d bytes", sizeof(trigger_buf)));
        }
        self->triggers[0].on_error = (trigger_buf[0] & 0x80U) != 0;
        self->triggers[0].on_tx = (trigger_buf[0] & 0x40U) != 0;
        self->triggers[0].on_rx = (trigger_buf[0] & 0x40U) == 0; // This bit is a "not on RX" bit

        uint8_t *can_data_mask = (uint8_t *)self->triggers[0].can_data_mask;
        uint8_t *can_data_match = (uint8_t *)self->triggers[0].can_data_match;
        for (uint8_t i = 0; i < 8U; i++) {
            can_data_mask[i] = trigger_buf[i + 11U];
            can_data_match[i] = trigger_buf[i + 19U];
        }
        self->triggers[0].can_dlc_mask = trigger_buf[9];
        self->triggers[0].can_dlc_match = trigger_buf[10];
        uint32_t id_word = BIG_ENDIAN_WORD(trigger_buf + 1U);
        // For standard IDs, the 11-bit ID is in bits 28:18, so normalize this to LSB-aligned
        bool ide_match = id_word & (1U << 29U);
        if (ide_match) {
            // ID word is already LSB-aligned, but needs masking to get the arbitration ID
            id_word &= 0x1ffffffU;
        }
        else {
            // Needs shifting and masking
            id_word = (id_word >> 18) & 0x7ffU;
        }

        self->triggers[0].arbitration_id_mask = id_word;
        self->triggers[0].arbitration_id_match = id_word;
        self->triggers[0].ide_match = ide_match;
        self->triggers[0].enabled = true;
    }
    else if (mp_on_canid != mp_const_none) {
        if (!MP_OBJ_IS_TYPE(mp_on_canid, &rp2_canid_type)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "on_canid must be of type CANID"));
        }
        if (as_bytes != mp_const_none) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Cannot set an ID trigger and a binary trigger"));
        }
        // Set masks to allow all data and sizes
        self->triggers[0].can_data_match[0] = 0;
        self->triggers[0].can_data_match[1] = 0;
        self->triggers[0].can_data_mask[0] = 0;
        self->triggers[0].can_data_mask[1] = 0;
        self->triggers[0].can_dlc_match = 0;
        self->triggers[0].can_dlc_mask = 0;
        // Set ID trigger
        self->triggers[0].arbitration_id_mask = 0x1fffffffU;
        self->triggers[0].arbitration_id_match = mp_on_canid->arbitration_id;
        self->triggers[0].ide_match = mp_on_canid->extended;
        self->triggers[0].on_rx = on_rx;
        self->triggers[0].on_tx = on_tx;
        self->triggers[0].enabled = true;
    }
    if (on_error) {
        self->triggers[0].on_error = on_error;
        self->triggers[0].enabled = true;
    }
    
    // Set the trigger pin on the CANPico as a GPIO port, drive low
    gpio_set_function(TRIG_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(TRIG_GPIO, GPIO_OUT);
    // Drive to 0
    gpio_clr_mask(1U << TRIG_GPIO);

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

STATIC TIME_CRITICAL void pulse_trigger(void)
{
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
}

// Put a pulse on the trigger pin
STATIC mp_obj_t rp2_can_pulse_trigger(mp_obj_t self_in)
{
    // Not used
    // rp2_can_obj_t *self = self_in;
    pulse_trigger();
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_pulse_trigger_obj, rp2_can_pulse_trigger);

//////////////////////////////////////// TEST FUNCTIONS /////////////////////////////////////////

STATIC mp_obj_t rp2_can_test_irq_init(void)
{
    CAN_DEBUG_PRINT("Enabling GPIO IRQ\n");
    gpio_set_irq_enabled(SPI_IRQ, EDGE_SENSITIVE_RISING, true);
    CAN_DEBUG_PRINT("Enabled GPIO IRQ and vector\n");
    CAN_DEBUG_PRINT("Set IRQ handler\n");
    CAN_DEBUG_PRINT("IRQ enabled\n");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_irq_init_fun_obj, rp2_can_test_irq_init);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_irq_init_obj, MP_ROM_PTR(&rp2_can_test_irq_init_fun_obj));

STATIC mp_obj_t rp2_can_test_irq_enable(void)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    mcp25xxfd_spi_gpio_enable_irq(&canpico_spi);
    CAN_DEBUG_PRINT("DISABLE_GPIO_INTERRUPTS() called\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_irq_enable_fun_obj, rp2_can_test_irq_enable);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_irq_enable_obj, MP_ROM_PTR(&rp2_can_test_irq_enable_fun_obj));

STATIC mp_obj_t rp2_can_test_irq_disable(void)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    mcp25xxfd_spi_gpio_disable_irq(&canpico_spi);
    CAN_DEBUG_PRINT("ENABLE_GPIO_INTERRUPTS() called\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_irq_disable_fun_obj, rp2_can_test_irq_disable);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_irq_disable_obj, MP_ROM_PTR(&rp2_can_test_irq_disable_fun_obj));

// FIXME remove when done

int can_debug_print(char *desc, uint32_t n)
{
    return mp_printf(MP_PYTHON_PRINTER, "%s=0x%08"PRIx32"\n", desc, n);
}

STATIC mp_obj_t rp2_can_test_spi_init(void)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    mcp25xxfd_spi_gpio_disable_irq(&canpico_spi);
    mcp25xxfd_spi_pins_init(&canpico_spi);
    mcp25xxfd_spi_gpio_enable_irq(&canpico_spi);
    CAN_DEBUG_PRINT("VTOR=0x%08"PRIx32"\n", scb_hw->vtor);
    for(int i = -16; i < 0x40; i++) {
        CAN_DEBUG_PRINT("VTOR[%d]=0x%08"PRIx32"\n", i, ((uint32_t *)(scb_hw->vtor))[i + 16]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_spi_init_fun_obj, rp2_can_test_spi_init);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_init_obj, MP_ROM_PTR(&rp2_can_test_spi_init_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_set(void)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    mcp25xxfd_spi_select(&canpico_spi);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_spi_set_fun_obj, rp2_can_test_spi_set);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_set_obj, MP_ROM_PTR(&rp2_can_test_spi_set_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_deselect(void)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    mcp25xxfd_spi_deselect(&canpico_spi);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(rp2_can_test_spi_deselect_fun_obj, rp2_can_test_spi_deselect);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_deselect_obj, MP_ROM_PTR(&rp2_can_test_spi_deselect_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_write_word(mp_obj_t addr_obj, mp_obj_t word_obj)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    uint32_t addr = mp_obj_get_int(addr_obj);
    uint32_t word = mp_obj_get_int(word_obj);

    CAN_DEBUG_PRINT("Writing word=0x%08"PRIx32"\n", word);
    mcp25xxfd_spi_gpio_disable_irq(&canpico_spi);
    uint8_t buf[6];
    // MCP25xxFD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x20 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;
    buf[2] = word & 0xffU;
    buf[3] = (word >> 8) & 0xffU;
    buf[4] = (word >> 16) & 0xffU;
    buf[5] = (word >> 24) & 0xffU;

    // SPI transaction
    // The Pico is little-endian so the first byte sent is the lowest-address, which is the
    // same as the RP2040
    mcp25xxfd_spi_select(&canpico_spi);
    mcp25xxfd_spi_write(&canpico_spi, buf, sizeof(buf));
    mcp25xxfd_spi_deselect(&canpico_spi);
    mcp25xxfd_spi_gpio_enable_irq(&canpico_spi);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(rp2_can_test_spi_write_word_fun_obj, rp2_can_test_spi_write_word);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_write_word_obj, MP_ROM_PTR(&rp2_can_test_spi_write_word_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_read_word(mp_obj_t addr_obj)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    uint32_t addr = mp_obj_get_int(addr_obj);

    mcp25xxfd_spi_gpio_disable_irq(&canpico_spi);
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
    mcp25xxfd_spi_select(&canpico_spi);
    mcp25xxfd_spi_read_write(&canpico_spi, cmd, resp, sizeof(cmd));
    mcp25xxfd_spi_deselect(&canpico_spi);

    uint32_t word = ((uint32_t)resp[2]) | ((uint32_t)resp[3] << 8) | ((uint32_t)resp[4] << 16) | ((uint32_t)resp[5] << 24);
    mcp25xxfd_spi_gpio_enable_irq(&canpico_spi);

    CAN_DEBUG_PRINT("Read word=0x%08"PRIx32"\n", word);
    return mp_obj_new_int_from_ull(word);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_test_spi_read_word_fun_obj, rp2_can_test_spi_read_word);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_read_word_obj, MP_ROM_PTR(&rp2_can_test_spi_read_word_fun_obj));

STATIC mp_obj_t rp2_can_test_spi_read_words(mp_obj_t addr_obj)
{
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    uint32_t words[4];
    uint32_t addr = mp_obj_get_int(addr_obj);

    uint8_t buf[2];

    // MCP25xxFD SPI transaction = command/addr, 4 bytes
    buf[0] = 0x30 | ((addr >> 8U) & 0xfU);
    buf[1] = addr & 0xffU;

    mcp25xxfd_spi_gpio_disable_irq(&canpico_spi);
    // SPI transaction
    mcp25xxfd_spi_select(&canpico_spi);
    // Send command, which flushes the pipeline then resumes
    mcp25xxfd_spi_write(&canpico_spi, buf, 2U);
    // Bulk data
    mcp25xxfd_spi_read(&canpico_spi, (uint8_t *)(words), 16U);
    mcp25xxfd_spi_deselect(&canpico_spi);
    mcp25xxfd_spi_gpio_enable_irq(&canpico_spi);

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
    can_interface_t canpico_spi;
    mcp25xxfd_spi_bind_canpico(&canpico_spi);

    uint32_t addr = mp_obj_get_int(addr_obj);
    uint32_t words[4] = {0xdeadbeefU, 0xcafef00dU, 0x01e551caU, 0x01020304U};

    // Prepare a contiguous buffer for the command because the SPI hardware is pipelined and do not want to stop
    // to switch buffers
    uint8_t cmd[18];
    // MCP25xxFD SPI transaction = command/addr, 4 bytes
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
    mcp25xxfd_spi_select(&canpico_spi);
    mcp25xxfd_spi_write(&canpico_spi, cmd, sizeof(cmd));
    mcp25xxfd_spi_deselect(&canpico_spi);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_can_test_spi_write_words_fun_obj, rp2_can_test_spi_write_words);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_can_test_spi_write_words_obj, MP_ROM_PTR(&rp2_can_test_spi_write_words_fun_obj));


STATIC void rp2_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
     rp2_can_obj_t *self = self_in;
     can_controller_t *controller = &self->controller;

    can_status_t status = can_get_status(controller);
    uint32_t timestamp_timer = can_get_time(controller);

    // Show the bus off status, the error passive status, TEC, REC, the time, and the baud rate settings
    mp_printf(print, "CAN(mode=");
    if (self->controller.mode == CAN_MODE_OFFLINE) {
        mp_printf(print, "CAN_MODE_OFFLINE");
    }
    else if (self->controller.mode == CAN_MODE_ACK_ONLY) {
        mp_printf(print, "CAN_MODE_ACK_ONLY");
    }
    else if (self->controller.mode == CAN_MODE_LISTEN_ONLY) {
        mp_printf(print, "CAN_MODE_LISTEN_ONLY");
    }
    else if (self->controller.mode == CAN_MODE_NORMAL) {
        mp_printf(print, "CAN_MODE_NORMAL");
    }
    else {
        mp_printf(print, "?");
    }

    uint16_t options = can_controller_get_options(&self->controller);

    if (options & CAN_OPTION_RECV_ERRORS) {
        mp_printf(print, ", recv_errors=True");
    }
    if (options & CAN_OPTION_REJECT_REMOTE) {
        mp_printf(print, ", reject_remote=True");
    }

    mp_printf(print, ", time=%lu, TEC=%d, REC=%d", timestamp_timer, can_status_get_tec(status), can_status_get_rec(status));

    // Error states
    if (can_status_is_bus_off(status))  {
        mp_printf(print, ", Bus Off");
    }
    if (can_status_is_error_passive(status)) {
        mp_printf(print, ", Error Passive");
    }
    if (can_status_is_error_warn(status)) {
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
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_500K_875), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_500K_875) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_250K_875), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_250K_875) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_125K_875), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_125K_875) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BITRATE_1M_875), MP_OBJ_NEW_SMALL_INT(CAN_BITRATE_1M_875) },
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
    { MP_OBJ_NEW_QSTR(MP_QSTR_RX_FIFO_SIZE), MP_OBJ_NEW_SMALL_INT(CAN_RX_FIFO_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_FIFO_SIZE), MP_OBJ_NEW_SMALL_INT(CAN_TX_FIFO_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_QUEUE_SIZE), MP_OBJ_NEW_SMALL_INT(CAN_TX_QUEUE_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TX_EVENT_FIFO_SIZE), MP_OBJ_NEW_SMALL_INT(CAN_TX_EVENT_FIFO_SIZE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_OVERFLOW), MP_OBJ_NEW_SMALL_INT(CAN_EVENT_TYPE_OVERFLOW) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_CAN_ERROR), MP_OBJ_NEW_SMALL_INT(CAN_EVENT_TYPE_CAN_ERROR) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_TRANSMITTED_FRAME), MP_OBJ_NEW_SMALL_INT(CAN_EVENT_TYPE_TRANSMITTED_FRAME) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EVENT_TYPE_RECEIVED_FRAME), MP_OBJ_NEW_SMALL_INT(CAN_EVENT_TYPE_RECEIVED_FRAME) },
};
STATIC MP_DEFINE_CONST_DICT(rp2_can_locals_dict, rp2_can_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_can_type,
    MP_QSTR_CAN,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_can_make_new,
    print, rp2_can_print,
    locals_dict, &rp2_can_locals_dict
    );

////////////////////////////////////// End of CAN class //////////////////////////////////////

////////////////////////////////// Start of CANFrame class ///////////////////////////////////
STATIC mp_obj_t rp2_canframe_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_canid,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},
        {MP_QSTR_data,      MP_ARG_OBJ,                   {.u_obj = mp_const_none}},
        {MP_QSTR_remote,    MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_tag,       MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0}},
        {MP_QSTR_dlc,       MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canid_obj_t *mp_canid = args[0].u_obj;
    mp_obj_t mp_data = args[1].u_obj;
    bool remote = args[2].u_bool;
    uint32_t tag = args[3].u_int;
    bool dlc_set = args[4].u_int != -1;
    uint8_t dlc = args[4].u_int;

    if (!MP_OBJ_IS_TYPE(mp_canid, &rp2_canid_type)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "canid must be of type CANID"));
    }
    if (dlc_set && dlc > 15) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "dlc must be 0..15"));
    }
    if (dlc_set && !remote && mp_data == mp_const_none && dlc > 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "dlc must be 0 if no data"));
    }

    uint8_t frame_dlc;
    uint32_t data_buf[2];
    if(mp_data == mp_const_none) {
        if (remote) {
            frame_dlc = dlc_set ? dlc : 0;
        }
        else {
            frame_dlc = 0;
        }
    }
    else {
        if(remote) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Remote frames cannot have data"));
        }
        uint8_t len = (uint8_t)copy_mp_bytes(mp_data, (uint8_t *)data_buf, 8U);
        // If there are insufficient bytes to match the DLC then this is an error
        if (dlc_set && len < 8U && dlc > len) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "dlc exceeds data length"));
        }
        frame_dlc = dlc_set ? dlc : len;
    }
    
    rp2_canframe_obj_t *self = m_new_obj(rp2_canframe_obj_t);
    self->base.type = &rp2_canframe_type;

    // Fill in the details of the CAN API frame
    can_make_frame(&self->frame, mp_canid->extended, mp_canid->arbitration_id, frame_dlc, (uint8_t *)data_buf, remote);
    // The reference in the frame is to the enclosing MicroPython CANFrame instance
    can_frame_set_uref(&self->frame, self);
    self->timestamp_valid = false;
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

    if ((bufinfo.len % FRAME_FROM_BYTES_NUM) != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Frames must be a multiple of %d bytes", FRAME_FROM_BYTES_NUM));
    }
    size_t num_frames = bufinfo.len / FRAME_FROM_BYTES_NUM;
    uint8_t *buf_ptr = bufinfo.buf;
    mp_obj_list_t *list = mp_obj_new_list(num_frames, NULL);

    for (uint32_t i = 0; i < num_frames; i++) {
        rp2_canframe_obj_t *self = m_new_obj(rp2_canframe_obj_t);
        self->base.type = &rp2_canframe_type;

        // Turn the bytes into a CAN frame (this stores the tag in uref)
        can_make_frame_from_bytes(&self->frame, buf_ptr);
        // Take the tag out of uref and store it in the CANFrame instance
        self->tag = (uint32_t)(can_frame_get_uref(&self->frame).ref); 
        // Set the uref to point to the CANFrame instance
        can_frame_set_uref(&self->frame, self);
        
        self->timestamp_valid = false;
        list->items[i] = self;
        buf_ptr += FRAME_FROM_BYTES_NUM;
    }

    return list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_from_bytes_fun_obj, rp2_canframe_from_bytes);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(rp2_canframe_from_bytes_obj, MP_ROM_PTR(&rp2_canframe_from_bytes_fun_obj));

// Get a byte representation of the frame
//
// Converts the frame into a byte representation that can be passed in to the
// static method for frame creation (i.e. round-tripping). Can be used to move
// CAN frames over other networks (e.g. WiFi)
STATIC mp_obj_t rp2_canframe_to_bytes(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    // Buffer, in same format as 'from bytes' static method
    uint8_t to_bytes[FRAME_FROM_BYTES_NUM];

    can_make_bytes_from_frame(to_bytes, &self->frame, self->tag);

    return make_mp_bytes(to_bytes, FRAME_FROM_BYTES_NUM);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_to_bytes_obj, rp2_canframe_to_bytes);

// Get the data as bytes
STATIC mp_obj_t rp2_canframe_get_data(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    uint8_t *data = can_frame_get_data(&self->frame);
    size_t len = can_frame_get_data_len(&self->frame);

    return make_mp_bytes(data, len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_data_obj, rp2_canframe_get_data);

STATIC mp_obj_t rp2_canframe_get_dlc(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(can_frame_get_dlc(&self->frame));
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

    return MP_OBJ_NEW_SMALL_INT(can_frame_get_id_filter(&self->frame));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_index_obj, rp2_canframe_get_index);

// Returns True if the frame has an extended ID, False otherwise
STATIC mp_obj_t rp2_canframe_is_extended(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return can_frame_is_extended(&self->frame) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_is_extended_obj, rp2_canframe_is_extended);

// Returns True if the frame is a remote frame, False otherwise
STATIC mp_obj_t rp2_canframe_is_remote(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return can_frame_is_remote(&self->frame) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_is_remote_obj, rp2_canframe_is_remote);

// Get a CANID instance representing the frame's CAN ID
STATIC mp_obj_t rp2_canframe_get_canid(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;
    rp2_canid_obj_t *mp_canid = m_new_obj(rp2_canid_obj_t);
    mp_canid->base.type = &rp2_canid_type;
    mp_canid->arbitration_id = can_frame_get_arbitration_id(&self->frame);
    mp_canid->extended = can_frame_is_extended(&self->frame);

    return mp_canid;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_canid_obj, rp2_canframe_get_canid);

// Get a numeric value for the CAN ID, with 11-bit IDs in the range 0..7ff
STATIC mp_obj_t rp2_canframe_get_arbitration_id(mp_obj_t self_in)
{
    rp2_canframe_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(can_frame_get_arbitration_id(&self->frame));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canframe_get_arbitration_id_obj, rp2_canframe_get_arbitration_id);

STATIC void rp2_canframe_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canframe_obj_t *self = self_in;
    can_frame_t *frame = &self->frame;

    mp_printf(print, "CANFrame(CANID(id=");
    if (can_frame_is_extended(frame)) {
        mp_printf(print, "E%08"PRIx32"", can_frame_get_arbitration_id(frame));
    }
    else {
        mp_printf(print, "S%03"PRIx32"", can_frame_get_arbitration_id(frame));
    }

    mp_printf(print, "), dlc=%d, data=", can_frame_get_dlc(frame));

    if(can_frame_is_remote(frame)) {
        mp_printf(print, "R");
    }
    else {
        size_t len = can_frame_get_data_len(frame);
        uint8_t *data = can_frame_get_data(frame);
        if(len) {
            for (uint32_t i = 0; i < len; i++) {
                mp_printf(print, "%02x", data[i]);
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
    { MP_OBJ_NEW_QSTR(MP_QSTR_to_bytes), (mp_obj_t)&rp2_canframe_to_bytes_obj },
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
    // Constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_FROM_BYTES_NUM), MP_OBJ_NEW_SMALL_INT(FRAME_FROM_BYTES_NUM) },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canframe_locals_dict, rp2_canframe_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_canframe_type,
    MP_QSTR_CANFrame,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_canframe_make_new,
    print, rp2_canframe_print,
    locals_dict, &rp2_canframe_locals_dict
    );

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
        if ((arbitration_id < 0) || (arbitration_id > CAN_ID_ARBITRATION_ID)) {
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
    self->arbitration_id = arbitration_id;
    self->extended = extended;

    return self;
}

// Get a numeric value for the CAN ID, with 11-bit IDs in the range 0..7ff
STATIC mp_obj_t rp2_canid_get_arbitration_id(mp_obj_t self_in)
{
    rp2_canid_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(self->arbitration_id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canid_get_arbitration_id_obj, rp2_canid_get_arbitration_id);

// Returns True if the ID is an extended ID, False otherwise
STATIC mp_obj_t rp2_canid_is_extended(mp_obj_t self_in)
{
    rp2_canid_obj_t *self = self_in;

    return self->extended ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canid_is_extended_obj, rp2_canid_is_extended);

// Get a CANID instance that will filter this ID
STATIC mp_obj_t rp2_canid_get_id_filter(mp_obj_t self_in)
{
    rp2_canid_obj_t *self = self_in;
    rp2_canidfilter_obj_t *mp_filter = m_new_obj(rp2_canidfilter_obj_t);
    mp_filter->base.type = &rp2_canidfilter_type;

    can_id_t canid = can_make_id(self->extended, self->arbitration_id);
    can_make_id_filter(&mp_filter->filter, canid);

    return mp_filter;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canid_get_id_filter_obj, rp2_canid_get_id_filter);

STATIC void rp2_canid_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canid_obj_t *self = self_in;

    if (self->extended) {
        mp_printf(print, "CANID(id=E%03"PRIx32")", self->arbitration_id);
    }
    else {
        mp_printf(print, "CANID(id=S%03"PRIx32")", self->arbitration_id);
    }
}

STATIC const mp_map_elem_t rp2_canid_locals_dict_table[] = {
    // Instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_arbitration_id), (mp_obj_t)&rp2_canid_get_arbitration_id_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_is_extended), (mp_obj_t)&rp2_canid_is_extended_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_id_filter), (mp_obj_t)&rp2_canid_get_id_filter_obj },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canid_locals_dict, rp2_canid_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_canid_type,
    MP_QSTR_CANID,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_canid_make_new,
    print, rp2_canid_print,
    locals_dict, &rp2_canid_locals_dict
    );

////////////////////////////////////// End of CANID class //////////////////////////////////////

///////////////////////////////// Start of CANIDFilter class ///////////////////////////////////
// Create the CANID instance and initialize the controller
STATIC mp_obj_t rp2_canidfilter_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_filter_str,     MP_ARG_OBJ, {.u_obj = mp_const_none}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t filter_str = args[0].u_obj;

    rp2_canidfilter_obj_t *self = m_new_obj(rp2_canidfilter_obj_t);
    self->base.type = &rp2_canidfilter_type;

    // If no filter string is specified then the filter become a "match all". This can be placed
    // at the end of a filter list so that the earlier filters can be used to identify an incoming frame
    // because this one will always match.
    if (filter_str == mp_const_none) {
        // Set up a filter for "accept all"
        can_make_id_filter_all(&self->filter);
    }
    else {
        if (!MP_OBJ_IS_STR_OR_BYTES(filter_str)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Filter must be a string or bytes"));
        }

        mp_buffer_info_t bufinfo;
        uint8_t data[1];
        rp2_buf_get_for_send(filter_str, &bufinfo, data);

        if (bufinfo.len != 29U && bufinfo.len != 11U) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Filter must be 11 or 29 characters long"));
        }

        uint32_t mask = 0;
        uint32_t match = 0;
        bool extended = bufinfo.len == 29U;

        for (mp_uint_t i = 0; i < bufinfo.len; i++) {
            char ch = (((byte *) bufinfo.buf)[i]);

            mask <<= 1;
            match <<= 1;
            
            switch(ch) {
                case '1':
                    // Must match 1
                    mask |= 1U;
                    match |= 1U;
                    break;
                case '0':
                    // Must match 0
                    mask |= 1U;
                    match |= 0U;
                    break;
                case 'X':
                    // Already both are zero
                    break;
                default:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Illegal character in filter: must be '1', '0' or 'X'"));
                    break; // Not reached
            }
        }
        can_make_id_filter_masked(&self->filter, extended, match, mask);
    }
    return self;
}

STATIC void rp2_canidfilter_print_mask(const mp_print_t *print, bool extended, uint32_t mask, uint32_t match)
{
    uint32_t size;

    if (extended) {
        mask <<= 3;
        match <<= 3;
        size = 29U;
    }
    else {
        mask <<= 21;
        match <<= 21;
        size = 11U;
    };

    for(uint32_t i = 0; i < size; i++) {
        if (mask & 0x80000000U) {
            // Must match
            if (match & 0x80000000U) {
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
        mask <<= 1;
        match <<= 1;
    }

}

STATIC void rp2_canidfilter_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canidfilter_obj_t *self = self_in;
    can_id_filter_t *filter = &self->filter;

    mp_printf(print, "CANIDFilter(filter=");

    if (can_id_filter_is_all(filter)) {
        mp_printf(print, "*");
    }
    else {
        bool extended = can_id_filter_is_extended(filter);
        uint32_t mask = can_id_filter_get_mask(filter);
        uint32_t match = can_id_filter_get_match(filter);

        rp2_canidfilter_print_mask(print, extended, mask, match);
    }

    mp_printf(print, ")");
}

STATIC const mp_map_elem_t rp2_canidfilter_locals_dict_table[] = {
    // Instance methods
};
STATIC MP_DEFINE_CONST_DICT(rp2_canidfilter_locals_dict, rp2_canidfilter_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_canidfilter_type,
    MP_QSTR_CANIDFilter,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_canidfilter_make_new,
    print, rp2_canidfilter_print,
    locals_dict, &rp2_canidfilter_locals_dict
    );

////////////////////////////////////// End of CANIDFilter class //////////////////////////////////////

////////////////////////////////////// Start of CANError class //////////////////////////////////////
// Create the CANError instance
STATIC mp_obj_t rp2_canerror_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_details,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t details = args[0].u_int;

    can_error_t error;
    error.details = details;

    rp2_canerror_obj_t *self = m_new_obj(rp2_canerror_obj_t);
    self->base.type = &rp2_canerror_type;
    self->error = error;
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
    can_error_t *e = &self->error;

    return can_error_is_crc(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_crc_error_obj, rp2_canerror_is_crc_error);

STATIC mp_obj_t rp2_canerror_is_stuff_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    return can_error_is_stuff(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_stuff_error_obj, rp2_canerror_is_stuff_error);

STATIC mp_obj_t rp2_canerror_is_form_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    return can_error_is_form(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_form_error_obj, rp2_canerror_is_form_error);

STATIC mp_obj_t rp2_canerror_is_ack_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    return can_error_is_ack(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_ack_error_obj, rp2_canerror_is_ack_error);

STATIC mp_obj_t rp2_canerror_is_bit1_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    return can_error_is_bit1(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_bit1_error_obj, rp2_canerror_is_bit1_error);

STATIC mp_obj_t rp2_canerror_is_bit0_error(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    return can_error_is_bit0(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_bit0_error_obj, rp2_canerror_is_bit0_error);

STATIC mp_obj_t rp2_canerror_is_bus_off(mp_obj_t self_in)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    return can_error_is_bus_off(e) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canerror_is_bus_off_obj, rp2_canerror_is_bus_off);

STATIC void rp2_canerror_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canerror_obj_t *self = self_in;
    can_error_t *e = &self->error;

    mp_printf(print, "CANError(");
    bool prev_item = false;
    // Prints the error type
    if (can_error_is_bus_off(e)) {
        mp_printf(print, "bus_off=True");
        prev_item = true;
    }
    if (can_error_is_crc(e)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "crc_error=True");
        prev_item = true;
    }
    if (can_error_is_stuff(e)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "stuff_error=True");
        prev_item = true;
    }
    if (can_error_is_form(e)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "form_error=True");
        prev_item = true;
    }
    if (can_error_is_ack(e)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "ack_error=True");
        prev_item = true;
    }
    if (can_error_is_bit1(e)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "bit1_error=True");
        prev_item = true;
    }
    if (can_error_is_bit0(e)) {
        if (prev_item) {
            mp_printf(print, ", ");
        }
        mp_printf(print, "bit0_error=True");
    }
    if (prev_item) {
        mp_printf(print, ", ");
    }
    mp_printf(print, "frame_cnt=%d, timestamp=%lu)", can_error_get_frame_cnt(e), self->timestamp);
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

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_canerror_type,
    MP_QSTR_CANError,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_canerror_make_new,
    print, rp2_canerror_print,
    locals_dict, &rp2_canerror_locals_dict
    );

////////////////////////////////////// End of CANError class //////////////////////////////////////

///////////////////////////////////// Start of CANOverflow class //////////////////////////////////
// Create the CANOverflow instance
STATIC mp_obj_t rp2_canoverflow_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canoverflow_obj_t *self = m_new_obj(rp2_canoverflow_obj_t);
    self->base.type = &rp2_canoverflow_type;
    self->frame_cnt = 0;
    self->receive = true;
    self->error_cnt = 0;
    self->timestamp = 0;

    return self;
}

// Returns the overflow frame count
STATIC mp_obj_t rp2_canoverflow_get_timestamp(mp_obj_t self_in)
{
    rp2_canoverflow_obj_t *self = self_in;

    return mp_obj_new_int_from_uint(self->timestamp);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canoverflow_get_timestamp_obj, rp2_canoverflow_get_timestamp);

// Returns the overflow frame count
STATIC mp_obj_t rp2_canoverflow_get_frame_cnt(mp_obj_t self_in)
{
    rp2_canoverflow_obj_t *self = self_in;

    return mp_obj_new_int_from_uint(self->frame_cnt);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canoverflow_get_frame_cnt_obj, rp2_canoverflow_get_frame_cnt);

// Returns the overflow error count
STATIC mp_obj_t rp2_canoverflow_get_error_cnt(mp_obj_t self_in)
{
    rp2_canoverflow_obj_t *self = self_in;

    if (self->receive) {
        return mp_obj_new_int_from_uint(self->error_cnt);
    }
    else {
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canoverflow_get_error_cnt_obj, rp2_canoverflow_get_error_cnt);

STATIC void rp2_canoverflow_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_canoverflow_obj_t *self = self_in;
    if (self->receive) {
        mp_printf(print, "CANOverflow(frame_cnt=%d, error_cnt=%d, timestamp=%lu)", self->frame_cnt, self->error_cnt, self->timestamp);
    }
    else {
        mp_printf(print, "CANOverflow(frame_cnt=%d, timestamp=%lu)", self->frame_cnt, self->timestamp);
    }
}

STATIC const mp_map_elem_t rp2_canoverflow_locals_dict_table[] = {
        // Instance methods
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_frame_cnt), (mp_obj_t)&rp2_canoverflow_get_frame_cnt_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_error_cnt), (mp_obj_t)&rp2_canoverflow_get_error_cnt_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_timestamp), (mp_obj_t)&rp2_canoverflow_get_timestamp_obj },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canoverflow_locals_dict, rp2_canoverflow_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_canoverflow_type,
    MP_QSTR_CANOverflow,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_canoverflow_make_new,
    print, rp2_canoverflow_print,
    locals_dict, &rp2_canoverflow_locals_dict
    );

////////////////////////////////////// End of CANOverflow class ///////////////////////////////////

//////////////////////////////// Start of callbacks of CANOverflow class //////////////////////////

// Transmit ISR callback to track timestamp of the sent frame
void TIME_CRITICAL can_isr_callback_frame_tx(can_uref_t uref, uint32_t timestamp)
{
    // Called with interrupts locked

    // The uref contains a pointer to the CANFrame instance that was transmitted
    // so update its timestamp.    
    rp2_canframe_obj_t *mp_frame = (rp2_canframe_obj_t *)(uref.ref);
    mp_frame->timestamp = timestamp;
    mp_frame->timestamp_valid = true;

    can_frame_t *frame = &mp_frame->frame;
    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj[0]);
    // Guard against spurious interrupt callbacks
    if (self != MP_OBJ_NULL) {
        can_trigger_t *trigger = &self->triggers[0];
        uint32_t arbitration_id = can_frame_get_arbitration_id(frame);
        uint8_t dlc = can_frame_get_dlc(frame);

        // Check to see if the transmitted frame matches and should trigger
        if (trigger->enabled && trigger->on_tx) {
            if (((arbitration_id & trigger->arbitration_id_mask) == trigger->arbitration_id_match) &&
                ((dlc & trigger->can_dlc_mask) == self->triggers->can_dlc_match) &&
                ((frame->data[0] & trigger->can_data_mask[0]) == self->triggers->can_data_match[0]) &&
                ((frame->data[0] & trigger->can_data_mask[1]) == self->triggers->can_data_match[1])) {
                pulse_trigger();
            }
        }
    }
}

// Callback to convert a uref into 32-bit integer for returning in the as_bytes call
uint32_t TIME_CRITICAL can_isr_callback_uref(can_uref_t uref)
{
    // Called with interrupts locked

    // The user-reference for the CAN API is pointers to MicroPython CANFrame class instances,
    // which when turned into bytes should give a 32-bit application tag that resides in the CANFrame instance
    rp2_canframe_obj_t *mp_frame = (rp2_canframe_obj_t *)(uref.ref);
    uint32_t tag = mp_frame->tag; // Application-provided 32-bit tag
    return tag;
}

// Called when a frame is received. Process the trigger here.
void TIME_CRITICAL can_isr_callback_frame_rx(can_frame_t *frame, uint32_t timestamp)
{
    // Called with interrupts locked

    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj[0]);
    // Guard against spurious interrupt callbacks
    if (self != MP_OBJ_NULL) {
        can_trigger_t *trigger = &self->triggers[0];
        uint32_t arbitration_id = can_frame_get_arbitration_id(frame);
        uint8_t dlc = can_frame_get_dlc(frame);

        if (trigger->enabled && trigger->on_rx) {
            if (((arbitration_id & trigger->arbitration_id_mask) == trigger->arbitration_id_match) &&
                ((dlc & trigger->can_dlc_mask) == self->triggers->can_dlc_match) &&
                ((frame->data[0] & trigger->can_data_mask[0]) == self->triggers->can_data_match[0]) &&
                ((frame->data[0] & trigger->can_data_mask[1]) == self->triggers->can_data_match[1])) {
                pulse_trigger();
            }
        }

        // Potential callback to Python function (done after trigger because function could be slow)
        if (self->mp_rx_callback_fn != mp_const_none) {
            // Frame here is created in a global space and does NOT have a lifetime beyond the
            // callback.
            static rp2_canframe_obj_t mp_frame_tmp;
            mp_frame_tmp.frame = *frame;
            mp_frame_tmp.base.type = &rp2_canframe_type;
            mp_frame_tmp.tag = 0;
            mp_frame_tmp.timestamp = timestamp;
            mp_frame_tmp.timestamp_valid = true;

            // Already has been verified that this function is a callable Python function, so
            // hand it the CANFrame instance so the handler can inspect it and react quickly
            mp_sched_schedule(self->mp_rx_callback_fn, MP_OBJ_FROM_PTR(&mp_frame_tmp));
        }
    }
}

// Called when there is a CAN error. Process the trigger here.
void TIME_CRITICAL can_isr_callback_error(can_error_t error, uint32_t timestamp)
{
    // Called with interrupts locked

    rp2_can_obj_t *self = MP_STATE_PORT(rp2_can_obj[0]);

    // Guard against a spurious interrupt that is raised after the controller is stopped.
    if (self != MP_OBJ_NULL) {
        can_trigger_t *trigger = &self->triggers[0];
        
        if (trigger->enabled && trigger->on_error) {
            pulse_trigger();
        }
    }
}

// In the future there may be more than one CAN controller
MP_REGISTER_ROOT_POINTER(void *rp2_can_obj[1]);


