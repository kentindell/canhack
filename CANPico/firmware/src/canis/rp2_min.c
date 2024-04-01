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

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include "py/nlr.h"
#include "py/objtuple.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "tusb.h"
#include "ports/rp2/canis/common.h"
#include "rp2_min.h"


// MIN on the Pico uses a second USB serial port (0 is stdin/stdout and used for REPL), allocated to
// itf 1.

#if CFG_TUD_CDC != 2
#error "No second CDC for MIN"
#endif

// Second CDC port: corresponds to the descriptor index
#define MIN_CDC_ITF         (1U)

#ifndef TRANSPORT_PROTOCOL
#error "MIN transport protocol not enabled"
#endif

// TODO expose this call in the MIN class as a function for writing bytes to the second port,
//      which can be used as a performance check. Could pass in a list.

// Writes all the bytes requested to the USB, returns when done (this may be slow due to running
// in XIP flash).
STATIC void usb_write(size_t len, uint8_t *src)
{
    if (tud_cdc_n_connected(MIN_CDC_ITF)) {
        for (size_t i = 0; i < len;) {
            uint32_t n = len - i;
            if (n > CFG_TUD_CDC_EP_BUFSIZE) {
                n = CFG_TUD_CDC_EP_BUFSIZE;
            }
            while (n > tud_cdc_n_write_available(MIN_CDC_ITF)) {
                tud_task();
                tud_cdc_n_write_flush(MIN_CDC_ITF);
            }
            uint32_t n2 = tud_cdc_n_write(MIN_CDC_ITF, (const char *)(src + i), n);
            tud_task();
            tud_cdc_n_write_flush(MIN_CDC_ITF);
            i += n2;
        }
    }
}

// Read as many characters as possible from the USB
// TODO expose this as a class method to allow it to be used as a performance check (see above)
//      also could use a pre-allocated list that's overwritten rather than returning anything
//      using the heap.
STATIC uint32_t usb_read(uint8_t *dest, size_t max_len)
{
    if (tud_cdc_n_connected(MIN_CDC_ITF) && tud_cdc_n_available(MIN_CDC_ITF)) {
        return tud_cdc_n_read(MIN_CDC_ITF, dest, max_len);
    }
    return 0;
}

// Deinit the root pointer
void min_deinit(void) {
    // Called when the system is reset.
    MP_STATE_PORT(rp2_min_obj[0]) = MP_OBJ_NULL;
}


// Transport MIN implementation for Micropython
//
// One class: MIN
//
// T-MIN API:
// - send_frame() - creates a MIN frame and queues it in the outgoing queue
// - recv() - drive the state machine for sending and receiving
//

// Flags used for rp2mon bridge
// TODO refactor rp2mon into this MIN class

// Only a single MIN context is supported for the USB port
// TODO support multiple MIN ports

/////////////////// MIN CALLBACKS //////////////////

#ifdef MIN_DEBUG_PRINTING
void min_debug_print(const char *msg, ...)
{
    va_list vargs;
    va_start(vargs, msg);
    vprintf(msg, vargs);
    va_end (vargs);
}

STATIC void print_bytes(char *msg, uint8_t *bytes, uint16_t n)
{
    printf(msg);
    for (uint16_t i = 0; i < n; i++) {
        printf("%02x", bytes[i]);
    }
    printf("\n");
}
#endif

uint32_t min_time_ms(void)
{
    return mp_hal_ticks_ms();
}

// Don't shortcut this: return a list of MIN messages
void min_application_handler(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload, uint8_t port)
{
    // Handles incoming MIN frames: accumulates the frames (as a 2-tuple) in a MicroPython list

    // All other MIN frames get added to a list to return to the Python caller to deal with
    rp2_min_obj_t *self = MP_STATE_PORT(rp2_min_obj[0]);
    if (self != MP_OBJ_NULL) {
        mp_obj_tuple_t *tuple = mp_obj_new_tuple(2, NULL);
        tuple->items[0] = MP_OBJ_NEW_SMALL_INT(min_id);
        tuple->items[1] = make_mp_bytes(min_payload, len_payload);
        // If list was empty so far (i.e. an empty tuple) then use the heap to create an empty list
        if (self->recv_list == mp_const_empty_tuple) {
            self->recv_list = mp_obj_new_list(0, NULL);
        }
        mp_obj_list_append(self->recv_list, tuple);
    }
}

// CALLBACK. Must return current buffer space in the given port. Used to check that a frame can be
// queued.
uint16_t min_tx_space(uint8_t port)
{
    // We can lie here because we are running outside ISRs and are prepared to spin on the
    // buffer until the frame is sent (which may of course lead instead to incoming CAN frames
    // and transmission reports to be lost instead but that's going to happen anyway.
    return 1024U;
}

void min_tx_byte(uint8_t port, uint8_t byte)
{
    rp2_min_obj_t *self = MP_STATE_PORT(rp2_min_obj[0]);

    if (self != MP_OBJ_NULL) {
        if(self->outgoing_cdc_buf_len < CDC_BUF_SIZE) {
            self->outgoing_cdc_buf[self->outgoing_cdc_buf_len++] = byte;
        }
    }
}

void min_tx_start(uint8_t port) {
    rp2_min_obj_t *self = MP_STATE_PORT(rp2_min_obj[0]);
    if (self != MP_OBJ_NULL) {
        self->outgoing_cdc_buf_len = 0;
    }
}

void min_tx_finished(uint8_t port) {
    rp2_min_obj_t *self = MP_STATE_PORT(rp2_min_obj[0]);
    if (self != MP_OBJ_NULL) {
        usb_write(self->outgoing_cdc_buf_len, self->outgoing_cdc_buf);
    }
}

void min_init(void) {
    // Set up the root pointer to a null controller object so that the memory is not allocated
    // until MIN is used.
    MP_STATE_PORT(rp2_min_obj[0]) = MP_OBJ_NULL;
}

//////////////////////////////// MicroPython bindings ////////////////////////////////

#ifdef MIN_DEBUG
#define MIN_DEBUG_PRINT(fmt, args...)       mp_printf(MP_PYTHON_PRINTER, fmt, ##args)
#else
#define MIN_DEBUG_PRINT(fmt, args...)       /* */
#endif

STATIC void rp2_min_init_helper(rp2_min_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_retransmit_timeout,      MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 50} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // We only run MIN on one port (USB CDC port 1), so hardwire it to zero
    // TODO could support multiple MIN instances in the future on multiple CDC ports
    min_init_context(&self->min_context, 0);
}

STATIC mp_obj_t rp2_min_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);

    // Create class instance for controller
    rp2_min_obj_t *self = MP_STATE_PORT(rp2_min_obj[0]);

    if (MP_STATE_PORT(rp2_min_obj[0]) == MP_OBJ_NULL) {
        // Newly create object (we don't want it created always because it's a fairly large object, with
        // large receive FIFO and this shouldn't be allocated until needed).
        self = m_new_obj(rp2_min_obj_t);
        self->base.type = &rp2_min_type;
        MP_STATE_PORT(rp2_min_obj[0]) = self;
    }
    else {
        self = MP_STATE_PORT(rp2_min_obj[0]);
    }

    // configure the object
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    rp2_min_init_helper(self, n_args, args, &kw_args);

    // At the moment we use only a single MIN object
    return self;
}

// TODO placeholder: should probably be replaced with something more meaningful in the future
STATIC void rp2_min_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    // TODO rp2_min_obj_t *self = self_in;
    mp_printf(print, "MIN()");
}

STATIC mp_obj_t rp2_min_send_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_min_id,  MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_payload, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    // parse args
    rp2_min_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (MP_STATE_PORT(rp2_min_obj[0]) != MP_OBJ_NULL) {
        // TODO replace with common copy bytes function
        // get the buffer to send from
        mp_buffer_info_t bufinfo;
        uint8_t data[1];
        rp2_buf_get_for_send(args[1].u_obj, &bufinfo, data);

        if (bufinfo.len > 255) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Frame too big"));
        }
        if (args[0].u_int > 0x3f) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "MIN ID out of range"));
        }

        // Uses the MIN transport layer to queue a frame, which ensures delivery
        bool success = min_queue_frame(&self->min_context, (uint8_t) args[0].u_int, bufinfo.buf, (uint8_t) bufinfo.len);

        return success ? mp_const_true : mp_const_false;
    }
    else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "MIN de-initialized"));
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_min_send_frame_obj, 1, rp2_min_send_frame);

STATIC mp_obj_t rp2_min_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    rp2_min_obj_t *self = pos_args[0];

    if (MP_STATE_PORT(rp2_min_obj[0]) != MP_OBJ_NULL) {
        // Zero the frames with an empty tuple; if there are some frames
        // then the frame handler will create a list to put them in
        self->recv_list = mp_const_empty_tuple;

        // Pull out all the bytes that have accumulated in the VCP buffer and pass to MIN.
        self->incoming_cdc_buf_len = usb_read(self->incoming_cdc_buf, sizeof(self->incoming_cdc_buf));
        min_poll(&self->min_context, self->incoming_cdc_buf, self->incoming_cdc_buf_len);

        return self->recv_list; // Returns a list of MIN frames that weren't handled automatically by the transport protocol
    }
    else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "MIN de-initialized"));
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_min_recv_obj, 1, rp2_min_recv);

// Remove MIN (and from the root pointers to allow it to be garbage collected)
STATIC mp_obj_t rp2_min_deinit(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    // Not used
    // rp2_min_obj_t *self = pos_args[0];

    // Remove the root pointer so it will be garbage collected eventually when the reference to
    // it itself garbage collected
    MP_STATE_PORT(rp2_min_obj[0]) = MP_OBJ_NULL;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_min_deinit_obj, 1, rp2_min_deinit);

STATIC mp_obj_t rp2_min_send_bytes(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_payload, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    // Not used
    // rp2_min_obj_t *self = pos_args[0];

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (MP_STATE_PORT(rp2_min_obj[0]) != MP_OBJ_NULL) {
        // get the buffer to send from
        mp_buffer_info_t bufinfo;
        uint8_t data[1];
        rp2_buf_get_for_send(args[0].u_obj, &bufinfo, data);

        if (bufinfo.len > 255) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Frame too big"));
        }

        MIN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Payload accepted, preparing to send to USB\n");
        usb_write(bufinfo.len, bufinfo.buf);
        MIN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Payload written\n");

        return mp_const_none;
    }
    else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "MIN de-initialized"));
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_min_send_bytes_obj, 1, rp2_min_send_bytes);

// Remove MIN (and from the root pointers to allow it to be garbage collected)
STATIC mp_obj_t rp2_min_recv_bytes(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    // Not used
    // rp2_min_obj_t *self = pos_args[0];

    uint8_t buf[64];

    if (MP_STATE_PORT(rp2_min_obj[0]) != MP_OBJ_NULL) {
        MIN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Preparing to read from USB\n");
        uint32_t len = usb_read(buf, sizeof(buf));
        MIN_DEBUG_PRINT(MP_PYTHON_PRINTER, "Read %d bytes from USB\n", len);

        return make_mp_bytes(buf, len);
    }
    else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "MIN de-initialized"));
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_min_recv_bytes_obj, 1, rp2_min_recv_bytes);

STATIC const mp_map_elem_t rp2_min_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_frame), (mp_obj_t)&rp2_min_send_frame_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv), (mp_obj_t)&rp2_min_recv_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_deinit), (mp_obj_t)&rp2_min_deinit_obj},
    // debugging methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_bytes), (mp_obj_t)&rp2_min_send_bytes_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv_bytes), (mp_obj_t)&rp2_min_recv_bytes_obj},
};
STATIC MP_DEFINE_CONST_DICT(rp2_min_locals_dict, rp2_min_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_min_type,
    MP_QSTR_MIN,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_min_make_new,
    print, rp2_min_print,
    locals_dict, &rp2_min_locals_dict
    );

MP_REGISTER_ROOT_POINTER(void *rp2_min_obj[1]);
