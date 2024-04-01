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
#include <canis/canhack.h>
#include <py/mperrno.h>
#include <py/stream.h>
#include <py/runtime.h>
#include "py/mphal.h"
#include "py/objstr.h"
#include "py/obj.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "rp2_canhack.h"
#include "common.h"

// Placeholders for code that might not run on a second core and will be shared with other code
// that generates interrupts
STATIC void inline disable_irq(void) {
    __asm__ __volatile__ ("cpsid i");
}

STATIC void inline enable_irq(void) {
    __asm__ __volatile__ ("cpsie i");
}

STATIC void inline buf_get_for_send(mp_obj_t o, mp_buffer_info_t *bufinfo, byte *tmp_data) {
    if (mp_obj_is_int(o)) {
        tmp_data[0] = mp_obj_get_int(o);
        bufinfo->buf = tmp_data;
        bufinfo->len = 1;
        bufinfo->typecode = 'B';
    } else {
        mp_get_buffer_raise(o, bufinfo, MP_BUFFER_READ);
    }
}

typedef struct _canhack_rp2_obj_t {
    mp_obj_base_t base;
    uint32_t bit_rate_kbps;
} canhack_rp2_obj_t;


// Construct a CAN hack object.
//
// For the CANHack and CANPico boards, the physical pins of the CAN transceiver are:
//
//  RP2040 GP22 = CAN TX = Pico pin 29
//  RP2040 GP21 = CAN RX = Pico pin 27
//
// For the CHV DEFCON30 badge, the physical pins of the CAN transceiver are:
//
// RP4040 GP17 = CAN TX = Pico pin 22
// RP2040 GP16 = CAN RX = Pico pin 21
//
// Select -DCHV_DEFCON30_BADGE to build for the CHV badge

// These ports need to be initialized as outputs.
//
// The timer being used will be the 16-bit free-running counter of PWM 7 (see RP2040 Datasheet section 4.4),
// a 16-bit counter value at CH7_CTR. The counter must be clocked at the full speed of the CPU.


// init(bit_time, sample_point)
STATIC mp_obj_t rp2_canhack_init_helper(canhack_rp2_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_extframe, ARG_prescaler, ARG_sjw, ARG_bs1, ARG_bs2, ARG_auto_restart };
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_bit_rate,     MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int  = 500} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t bit_rate = args[0].u_int;

    if (bit_rate != 500 && bit_rate != 250 && bit_rate != 125) {
    }

    self->bit_rate_kbps = bit_rate;
    init_gpio();

    switch (bit_rate) {
        case 500U:
            init_ctr(BAUD_500KBIT_PRESCALE);
            break;
        case 250U:
            init_ctr(BAUD_250KBIT_PRESCALE);
            break;
        case 125U:
            init_ctr(BAUD_125KBIT_PRESCALE);
            break;
        default:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Valid baud rates are 500, 250, 125 kbit/sec"));
            // NOTREACHED
    }
    canhack_init();

    SET_CAN_TX_REC();

    return mp_const_none;
}

STATIC mp_obj_t rp2_canhack_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return rp2_canhack_init_helper(MP_OBJ_TO_PTR(args[0]), n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_init_obj, 1, rp2_canhack_init);

// CANHack(...)
STATIC mp_obj_t rp2_canhack_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);

    canhack_rp2_obj_t *self = m_new_obj(canhack_rp2_obj_t);
    self->base.type = &rp2_canhack_type;

    // configure the object
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    rp2_canhack_init_helper(self, n_args, args, &kw_args);

    return self;
}

// Trivial function to stop the current attack (only useful in a multi-threaded environment
STATIC mp_obj_t rp2_canhack_stop(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    canhack_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_stop_obj, 1, rp2_canhack_stop);

STATIC mp_obj_t rp2_canhack_set_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_can_id,    MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int  = 0x7ff} },
            { MP_QSTR_remote,    MP_ARG_KW_ONLY | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_extended,  MP_ARG_KW_ONLY | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_data,      MP_ARG_OBJ,                    {.u_obj  = mp_const_none} },
            { MP_QSTR_set_dlc,   MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
            { MP_QSTR_dlc,       MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0} },
            { MP_QSTR_second,    MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
            { MP_QSTR_no_ack,    MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t can_id = args[0].u_int;
    bool rtr = args[1].u_bool;
    bool ide = args[2].u_bool;
    mp_obj_t data_obj = args[3].u_obj;
    bool set_dlc = args[4].u_bool;
    // args[5] dealt with below
    bool second = args[6].u_bool;
    bool no_ack = args[7].u_bool;

    uint32_t len;
    uint32_t dlc;
    uint8_t data[8];

    if(data_obj == mp_const_none) {
        len = 0;
    }
    else {
        len = copy_mp_bytes(data_obj, data, 8U);
    }

    // DLC can be set if remote, but must have no payload
    if(rtr && (len > 0)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Remote frames cannot have a payload"));
    }
    // 8 byte frames max
    if (len > 8U) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Payload cannot be more than 8 bytes"));
    }

    if (set_dlc) {
        dlc = args[5].u_int;
        // DLC cannot be > 15
        if (dlc > 15U) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "DLC must be <= 15"));
        }
    }
    else {
        dlc = len;
    }

    uint32_t id_a;
    uint32_t id_b;

    // Pull apart CAN ID field into ID A and B.
    if (ide) {
        id_a = (can_id >> 18U) & 0x7ffU;
        id_b = can_id & 0x3ffffU;
    }
    else {
        id_a = can_id & 0x7ffU;
        id_b =  0;
    }
    canhack_frame_t *frame = canhack_get_frame(second);
    canhack_set_frame(id_a, id_b, rtr, ide, dlc, data, frame);

    if (no_ack) {
        // If ACK=1 is wanted (i.e. a pure frame) then override the value created by the CANHack toolkit
        frame->tx_bitstream[frame->last_crc_bit + 2U] = 1U;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_set_frame_obj, 1, rp2_canhack_set_frame);

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"


STATIC mp_obj_t rp2_canhack_get_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_second,           MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool second = args[0].u_bool;
    uint8_t frame_bits[CANHACK_MAX_BITS];
    mp_obj_t frame_bytes;
    mp_obj_t stuff_bit_bytes;

    // Return a tuple of:
    // String for bits (including stuff bits)
    // String for bits (excluding stuff bits)
    // Integers for last bit of arbitration ID, etc.

    canhack_frame_t *frame = canhack_get_frame(second);

    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    for(uint32_t i = 0; i < frame->tx_bits; i++) {
        frame_bits[i] = frame->tx_bitstream[i] ? '1' : '0';
    }
    frame_bytes = make_mp_bytes(frame_bits, frame->tx_bits);

    for(uint32_t i = 0; i < frame->tx_bits; i++) {
        frame_bits[i] = frame->stuff_bit[i] ? 'X' : '-';
    }
    stuff_bit_bytes = make_mp_bytes(frame_bits, frame->tx_bits);

    mp_obj_tuple_t *tuple = mp_obj_new_tuple(8, NULL);
    tuple->items[0] = frame_bytes;
    tuple->items[1] = stuff_bit_bytes;
    tuple->items[2] = mp_obj_new_int_from_uint(frame->last_arbitration_bit);
    tuple->items[3] = mp_obj_new_int_from_uint(frame->last_dlc_bit);
    tuple->items[4] = mp_obj_new_int_from_uint(frame->last_data_bit);
    tuple->items[5] = mp_obj_new_int_from_uint(frame->last_crc_bit);
    tuple->items[6] = mp_obj_new_int_from_uint(frame->last_eof_bit);
    tuple->items[7] = mp_obj_new_int_from_uint(frame->crc_rg);

    return tuple;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_get_frame_obj, 1, rp2_canhack_get_frame);

STATIC mp_obj_t rp2_canhack_print_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_stuff_bits,           MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = true} },
            { MP_QSTR_second,               MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool stuff_bits = args[0].u_bool;
    bool second = args[1].u_bool;

    canhack_frame_t *frame = canhack_get_frame(second);

    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    char *colour = ANSI_COLOR_YELLOW;
    mp_printf(MP_PYTHON_PRINTER, "%s", colour);
    for (uint32_t i = 0; i < frame->tx_bits; i++) {
        char *bit_str = frame->tx_bitstream[i] ? "1" : "0";
        if (frame->stuff_bit[i]) {
            if (stuff_bits) {
                mp_printf(MP_PYTHON_PRINTER, "%s%s%s", ANSI_COLOR_RED, bit_str, colour);
            }
        }
        else {
            mp_printf(MP_PYTHON_PRINTER, "%s", bit_str);
        }
        if (i == frame->last_arbitration_bit) {
            colour = ANSI_COLOR_BLUE;
            mp_printf(MP_PYTHON_PRINTER, "%s", colour);
        }
        if (i == frame->last_dlc_bit) {
            colour = ANSI_COLOR_GREEN;
            mp_printf(MP_PYTHON_PRINTER, "%s", colour);
        }
        if (i == frame->last_data_bit) {
            colour = ANSI_COLOR_CYAN;
            mp_printf(MP_PYTHON_PRINTER, "%s", colour);
        }
        if (i == frame->last_crc_bit) {
            colour = ANSI_COLOR_RESET;
            mp_printf(MP_PYTHON_PRINTER, "%s", colour);
        }
    }
    mp_printf(MP_PYTHON_PRINTER, ANSI_COLOR_RESET "\n");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_print_frame_obj, 1, rp2_canhack_print_frame);

STATIC mp_obj_t rp2_canhack_send_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50000000U} },
            { MP_QSTR_second,            MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_retries,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_repeat,            MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 1U} }
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t timeout = args[0].u_int;
    bool second = args[1].u_bool;
    uint32_t retries = args[2].u_int;
    uint32_t repeat = args[3].u_int;

    canhack_frame_t *frame = canhack_get_frame(second);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    // Skip transmitting the EOF/IFS so (11 recessive bits) that can send frames back to back
    frame->tx_bits -= 11U;
    for(;;) {
        // Disable interrupts around the library call because any interrupts will mess up the timing
        disable_irq();
        RESET_CLOCK(0);
        // Transmit the frame with a timeout (default: 65K bit times, or about 130ms at 500kbit/sec)
        canhack_set_timeout(timeout);
        bool success = canhack_send_frame(retries, second);
        if (success) {
            repeat--;
        }
        enable_irq();
        if (!success || repeat == 0) {
            break;
        }
    }
    // Put the 11 recessive bits back
    frame->tx_bits += 11U;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_send_frame_obj, 1, rp2_canhack_send_frame);

STATIC mp_obj_t rp2_canhack_send_janus_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_sync_time,         MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50} },
            { MP_QSTR_split_time,        MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 155} },
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50000000U} },
            { MP_QSTR_retries,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Default to fractions of a bit time: 0-25% = sync time, 6.5%-62.5% = first bit, 62.5%-100% = second bit.
    uint32_t sync_time = args[0].u_int ? args[0].u_int : BIT_TIME / 4U;
    uint32_t split_time = args[1].u_int ? args[1].u_int : (BIT_TIME * 5U) / 8U;
    uint32_t timeout = args[2].u_int;
    uint32_t retries = args[3].u_int;

    canhack_frame_t *frame1 = canhack_get_frame(false);
    canhack_frame_t *frame2 = canhack_get_frame(true);
    if (!frame1->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }
    if (!frame2->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Second CAN frame has not been set"));
    }

    // Disable interrupts around the library call because any interrupts will mess up the timing
    disable_irq();
    // Transmit the frame with a timeout (default: 65K bit times, or about 130ms at 500kbit/sec)
    canhack_set_timeout(timeout);
    canhack_send_janus_frame(sync_time, split_time, retries);
    enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_send_janus_frame_obj, 1, rp2_canhack_send_janus_frame);

STATIC mp_obj_t rp2_canhack_spoof_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50000000U} },
            { MP_QSTR_overwrite,         MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_sync_time,         MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_split_time,        MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_second,            MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_retries,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_loopback_offset,   MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = DEFAULT_LOOPBACK_OFFSET} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t timeout = args[0].u_int;
    bool overwrite = args[1].u_bool;
    uint32_t sync_time = args[2].u_int ? args[2].u_int : BIT_TIME / 4U;
    uint32_t split_time = args[3].u_int ? args[3].u_int : (BIT_TIME * 5U) / 8U;
    bool second = args[4].u_bool;
    uint32_t retries = args[5].u_int;
    uint32_t loopback_offset = args[6].u_int;

    canhack_frame_t *frame = canhack_get_frame(false);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }
    if (second) {
        frame = canhack_get_frame(true);
        if (!frame->frame_set) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Second CAN frame has not been set"));
        }
    }

    // Target a frame
    canhack_set_attack_masks();

    if (overwrite) {
        // Disable interrupts around the library call because any interrupts will mess up the timing
        disable_irq();
        // Transmit the frame with a timeout. Target must be in error passive mode.
        canhack_set_timeout(timeout);

        for(;;) {
            bool ok = canhack_spoof_frame_error_passive(loopback_offset);
            // Repeat the attack, giving up if the timeout is hit or there was an error raised
            if (!ok || retries-- == 0) {
                break;
            }
        }
        enable_irq();
    }
    else {
        disable_irq();
        // Transmit a frame after detecting the target frame
        canhack_set_timeout(timeout);
        canhack_spoof_frame(second, sync_time, split_time, retries);
        enable_irq();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_spoof_frame_obj, 1, rp2_canhack_spoof_frame);

STATIC mp_obj_t rp2_canhack_error_attack(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_repeat,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2U} },
            { MP_QSTR_timeout,          MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50000000U} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t repeat = args[0].u_int;
    uint32_t timeout = args[1].u_int;

    canhack_frame_t *frame = canhack_get_frame(false);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    // Target a frame
    canhack_set_attack_masks();

    disable_irq();
    // Looking for 0111111 (targeting a bit in the error delimiter) to generate an error.
    canhack_set_timeout(timeout);
    bool timeout_occurred = canhack_error_attack(repeat, true, 0x7fU, 0x3fU);
    enable_irq();

    return timeout_occurred ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_error_attack_obj, 1, rp2_canhack_error_attack);

STATIC mp_obj_t rp2_canhack_double_receive_attack(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_repeat,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2U} },
            { MP_QSTR_timeout,          MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50000000U} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t repeat = args[0].u_int;
    uint32_t timeout = args[1].u_int;

    canhack_frame_t *frame = canhack_get_frame(false);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    // Target a frame
    canhack_set_attack_masks();

    disable_irq();
    do {
        // Looking for 01111111 (targeting last bit of EOF) to generate an error.
        canhack_set_timeout(timeout);
        canhack_error_attack(1U, false, 0xffU, 0x7fU);
    } while (repeat--);
    enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_double_receive_attack_obj, 1, rp2_canhack_double_receive_attack);

STATIC mp_obj_t rp2_canhack_freeze_doom_loop_attack(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_repeat,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2U} },
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 50000000U} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t repeat = args[0].u_int;
    uint32_t timeout = args[1].u_int;

    canhack_frame_t *frame = canhack_get_frame(false);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    // Target a frame
    canhack_set_attack_masks();

    disable_irq();
    // Looking for 011111111 (targeting first bit of IFS) to generate an overload.
    canhack_set_timeout(timeout);
    canhack_error_attack(repeat, false, 0x1ffU, 0x0ffU);
    enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_freeze_doom_loop_attack_obj, 1, rp2_canhack_freeze_doom_loop_attack);

STATIC mp_obj_t rp2_canhack_set_can_tx(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_recessive,           MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_int = true} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if(args[0].u_bool) {
        SET_CAN_TX_REC();
    }
    else {
        SET_CAN_TX_DOM();
    };

    if (GET_CAN_RX()) {
        return mp_const_true;
    }
    else {
        return mp_const_false;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_canhack_set_can_tx_obj, 1, rp2_canhack_set_can_tx);


STATIC mp_obj_t rp2_canhack_square_wave(mp_obj_t self_in)
{
    canhack_send_square_wave();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canhack_square_wave_obj, rp2_canhack_square_wave);


STATIC mp_obj_t rp2_canhack_loopback(mp_obj_t self_in)
{
    canhack_loopback();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canhack_loopback_obj, rp2_canhack_loopback);


STATIC mp_obj_t rp2_canhack_get_clock(mp_obj_t self_in)
{
    return mp_obj_new_int(GET_CLOCK());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canhack_get_clock_obj, rp2_canhack_get_clock);


STATIC mp_obj_t rp2_canhack_reset_clock(mp_obj_t self_in)
{
    ctr_t c = GET_CLOCK();
    RESET_CLOCK(0);

    return mp_obj_new_int(c);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canhack_reset_clock_obj, rp2_canhack_reset_clock);

ctr_t ts[160];

__attribute__((noinline, long_call, section(".time_critical"))) void send_raw_frame(canhack_frame_t *frame )
{
    uint8_t tx_index = 1U;
    uint8_t ts_index = 3U;
    ctr_t bit_end;
    uint8_t tx = frame->tx_bitstream[tx_index++];

    // SOF is first bit
    ts[0] = GET_CLOCK();
    RESET_CLOCK(0);
    ts[1] = GET_CLOCK();
    SET_CAN_TX_DOM();
    ts[2] = GET_CLOCK();
    bit_end = BIT_TIME;
    for (;;) {
        if (GET_CLOCK() >= bit_end) {
            SET_CAN_TX(tx);
            ts[ts_index++] = GET_CLOCK();
            tx = frame->tx_bitstream[tx_index++];
            if ((tx_index >= frame->tx_bits)) {
                SET_CAN_TX_REC();
                break;
            }
            bit_end += BIT_TIME;
        }
    }
}

STATIC mp_obj_t rp2_canhack_send_raw(mp_obj_t self_in)
{
    canhack_frame_t *frame = canhack_get_frame(false);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    disable_irq();
    ///////////////////
    send_raw_frame(frame);
    ///////////////////
    enable_irq();

    for(uint i = 0; i < frame->tx_bits; i++) {
        mp_printf(MP_PYTHON_PRINTER, "%d=%d (%d)\n", i, ts[i], ts[i] - BIT_TIME * (i - 2));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_canhack_send_raw_obj, rp2_canhack_send_raw);


STATIC const mp_map_elem_t rp2_canhack_locals_dict_table[] = {
        // instance methods
        { MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&rp2_canhack_init_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&rp2_canhack_stop_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_set_frame), (mp_obj_t)&rp2_canhack_set_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_frame), (mp_obj_t)&rp2_canhack_get_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_print_frame), (mp_obj_t)&rp2_canhack_print_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_send_frame), (mp_obj_t)&rp2_canhack_send_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_send_janus_frame), (mp_obj_t)&rp2_canhack_send_janus_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_spoof_frame), (mp_obj_t)&rp2_canhack_spoof_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_error_attack), (mp_obj_t)&rp2_canhack_error_attack_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_double_receive_attack), (mp_obj_t)&rp2_canhack_double_receive_attack_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_freeze_doom_loop_attack), (mp_obj_t)&rp2_canhack_freeze_doom_loop_attack_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_set_can_tx), (mp_obj_t)&rp2_canhack_set_can_tx_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_square_wave), (mp_obj_t)&rp2_canhack_square_wave_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_loopback), (mp_obj_t)&rp2_canhack_loopback_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_clock), (mp_obj_t)&rp2_canhack_get_clock_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_reset_clock), (mp_obj_t)&rp2_canhack_reset_clock_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_send_raw), (mp_obj_t)&rp2_canhack_send_raw_obj },
};
STATIC MP_DEFINE_CONST_DICT(rp2_canhack_locals_dict, rp2_canhack_locals_dict_table);

STATIC void rp2_canhack_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    canhack_rp2_obj_t *self = self_in;

    mp_printf(print, "CANHack(bit_rate=%d)", self->bit_rate_kbps);
}

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_canhack_type,
    MP_QSTR_CANHack,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_canhack_make_new,
    print, rp2_canhack_print,
    locals_dict, &rp2_canhack_locals_dict
    );

