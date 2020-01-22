//
// Created by ken on 11/12/2019.
//
// MicroPython bindings for the CAN hack module

#include <canhack.h>
#include <py/mperrno.h>
#include <py/stream.h>
#include <py/runtime.h>
#include "py/mphal.h"
#include "pin.h"
#include "bufhelper.h"
#include "py/objstr.h"
#include "py/obj.h"
#include "pyb_canhack.h"
#include "canhack-PYBV11.h"
#include <stdio.h>

uint32_t copy_mp_bytes(mp_obj_t *mp_bytes, uint8_t *dest, uint32_t max_len)
{
    if(!MP_OBJ_IS_STR_OR_BYTES(mp_bytes)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Bytes parameter expected"));
    }

    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    pyb_buf_get_for_send(mp_bytes, &bufinfo, data);

    if (bufinfo.len > max_len) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Bytes parameter too long"));
    }
    for (mp_uint_t i = 0; i < bufinfo.len; i++) {
        dest[i] = ((byte*)bufinfo.buf)[i];
    }

    return bufinfo.len;
}

typedef struct _pyb_canhack_obj_t {
    mp_obj_base_t base;
    uint32_t bit_time;
    uint32_t sample_point_offset;
} pyb_canhack_obj_t;

#define     BAUD_500KBIT        (336U)
#define     BAUD_250KBIT        (BAUD_500KBIT * 2U)
#define     BAUD_125KBIT        (BAUD_250KBIT * 2U)

// Construct a CAN hack object.
//
// The physical pins of the CAN bus is:
//
//  (RX, TX) = (Y3, Y4) = (PB8, PB9)

// init(bit_time, sample_point)
STATIC mp_obj_t pyb_canhack_init_helper(pyb_canhack_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_extframe, ARG_prescaler, ARG_sjw, ARG_bs1, ARG_bs2, ARG_auto_restart };
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_bit_time,     MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int  = BAUD_500KBIT} },
            { MP_QSTR_sample_point, MP_ARG_KW_ONLY | MP_ARG_INT,   {.u_int  = 50U} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t bit_time = args[0].u_int;
    uint32_t sample_point = args[1].u_int;

    if (bit_time < BAUD_500KBIT) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Not rated for baud rates higher than 500kbit/sec"));
    }
    if (sample_point > 75U) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Not rated for sample points higher than 75%"));
    }

    self->bit_time = bit_time;
    self->sample_point_offset = (bit_time * sample_point) / 100U;

    canhack_init(self->bit_time, self->sample_point_offset);

    // init GPIO
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = GPIO_PIN_8;                // CAN RX
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    SET_CAN_TX(CAN_TX_REC());
    GPIO_InitStructure.Pin = GPIO_PIN_9;                // CAN TX
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Set up the debug timer
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    return mp_const_none;
}

STATIC mp_obj_t pyb_canhack_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return pyb_canhack_init_helper(MP_OBJ_TO_PTR(args[0]), n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_init_obj, 1, pyb_canhack_init);

// CANHack(...)
STATIC mp_obj_t pyb_canhack_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);

    pyb_canhack_obj_t *self = m_new_obj(pyb_canhack_obj_t);
    self->base.type = &pyb_canhack_type;

    // configure the object
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    pyb_canhack_init_helper(self, n_args, args, &kw_args);

    return self;
}

STATIC mp_obj_t pyb_canhack_set_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_can_id,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0x7ff} },
            { MP_QSTR_remote,    MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
            { MP_QSTR_extended,  MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
            { MP_QSTR_data,      MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL} },
            { MP_QSTR_set_dlc,   MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_dlc,       MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_second,    MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t can_id = args[0].u_int;
    bool rtr = args[1].u_bool;
    bool ide = args[2].u_bool;
    mp_obj_t data_obj = args[3].u_obj;
    bool set_dlc = args[4].u_bool;

    bool second = args[6].u_bool;

    uint32_t len;
    uint32_t dlc;
    uint8_t data[8];

    if(data_obj == MP_OBJ_NULL) {
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

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_set_frame_obj, 1, pyb_canhack_set_frame);

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// Simple function to create Micropython bytes from a block of memory
STATIC mp_obj_t make_mp_bytes(const uint8_t *src, uint32_t len)
{
    vstr_t vstr;
    vstr_init_len(&vstr, len);
    for (mp_uint_t i = 0; i < len; i++) {
        vstr.buf[i] = src[i];
    }

    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC mp_obj_t pyb_canhack_get_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
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
        frame_bits[i] = IS_CAN_TX_REC(frame->tx_bitstream[i]) ? '1' : '0';
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
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_get_frame_obj, 1, pyb_canhack_get_frame);

STATIC mp_obj_t pyb_canhack_print_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
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
    printf("%s", colour);
    for (uint32_t i = 0; i < frame->tx_bits; i++) {
        char *bit_str = IS_CAN_TX_REC(frame->tx_bitstream[i]) ? "1" : "0";
        if (frame->stuff_bit[i]) {
            if (stuff_bits) {
                printf("%s%s%s", ANSI_COLOR_RED, bit_str, colour);
            }
        }
        else {
            printf("%s", bit_str);
        }
        if (i == frame->last_arbitration_bit) {
            colour = ANSI_COLOR_BLUE;
            printf("%s", colour);
        }
        if (i == frame->last_dlc_bit) {
            colour = ANSI_COLOR_GREEN;
            printf("%s", colour);
        }
        if (i == frame->last_data_bit) {
            colour = ANSI_COLOR_CYAN;
            printf("%s", colour);
        }
        if (i == frame->last_crc_bit) {
            colour = ANSI_COLOR_RESET;
            printf("%s", colour);
        }
    }
    printf(ANSI_COLOR_RESET "\n");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_print_frame_obj, 1, pyb_canhack_print_frame);

STATIC mp_obj_t pyb_canhack_send_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 65535U} },
            { MP_QSTR_second,            MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_retries,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t timeout = args[0].u_int;
    bool second = args[1].u_bool;
    uint32_t retries = args[2].u_int;

    canhack_frame_t *frame = canhack_get_frame(second);
    if (!frame->frame_set) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN frame has not been set"));
    }

    // Disable interrupts around the library call because any interrupts will mess up the timing
    __disable_irq();
    RESET_CPU_CLOCK();
    // Transmit the frame with a timeout (default: 65K bit times, or about 130ms at 500kbit/sec)
    canhack_send_frame(timeout, retries);
    __enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_send_frame_obj, 1, pyb_canhack_send_frame);

STATIC mp_obj_t pyb_canhack_send_janus_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    pyb_canhack_obj_t *self = pos_args[0];

    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_sync_time,         MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_split_time,        MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 65535U} },
            { MP_QSTR_retries,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Default to fractions of a bit time: 0-6.5% = sync time, 6.5%-62.5% = first bit, 62.5%-100% = second bit.
    uint32_t sync_time = args[0].u_int ? args[0].u_int : self->bit_time / 15U;
    uint32_t split_time = args[1].u_int ? args[1].u_int : (self->bit_time * 5U) / 8U;
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
    __disable_irq();
    // Transmit the frame with a timeout (default: 65K bit times, or about 130ms at 500kbit/sec)
    canhack_send_janus_frame(timeout, sync_time, split_time, retries);
    __enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_send_janus_frame_obj, 1, pyb_canhack_send_janus_frame);

STATIC mp_obj_t pyb_canhack_spoof_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2000000U} },
            { MP_QSTR_overwrite,         MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_sync_time,         MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_split_time,        MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
            { MP_QSTR_second,            MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_bool = false} },
            { MP_QSTR_retries,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = false} },
    };

    pyb_canhack_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t timeout = args[0].u_int;
    bool overwrite = args[1].u_bool;
    uint32_t sync_time = args[2].u_int ? args[2].u_int : self->bit_time / 4U;
    uint32_t split_time = args[3].u_int ? args[3].u_int : (self->bit_time * 5U) / 8U;
    bool second = args[4].u_bool;
    bool retries = args[5].u_int;

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
    canhack_set_attack_masks();    // Target a frame

    if (overwrite) {
        // Disable interrupts around the library call because any interrupts will mess up the timing
        __disable_irq();
        // Transmit the frame with a timeout. Target must be in error passive mode.
        canhack_spoof_frame_error_passive(timeout);
        __enable_irq();
    }
    else {
        __disable_irq();
        // Transmit a frame after detecting the target frame
        canhack_spoof_frame(timeout, second, sync_time, split_time, retries);
        __enable_irq();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_spoof_frame_obj, 1, pyb_canhack_spoof_frame);

STATIC mp_obj_t pyb_canhack_error_attack(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_repeat,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2U} },
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2000000U} },
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

    __disable_irq();
    // Looking for 0111111 (targeting a bit in the error delimiter) to generate an error.
    bool timeout_occurred = canhack_error_attack(timeout, repeat, true, 0x7fU, 0x3fU);
    __enable_irq();

    return timeout_occurred ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_error_attack_obj, 1, pyb_canhack_error_attack);

STATIC mp_obj_t pyb_canhack_double_receive_attack(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_repeat,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2U} },
            { MP_QSTR_timeout,          MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2000000U} },
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

    __disable_irq();
    do {
        // Looking for 01111111 (targeting last bit of EOF) to generate an error.
        canhack_error_attack(timeout, 1U, false, 0xffU, 0x7fU);
    } while (repeat--);
    __enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_double_receive_attack_obj, 1, pyb_canhack_double_receive_attack);

STATIC mp_obj_t pyb_canhack_freeze_doom_loop_attack(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_repeat,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2U} },
            { MP_QSTR_timeout,           MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 2000000U} },
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

    __disable_irq();
    // Looking for 011111111 (targeting first bit of IFS) to generate an overload.
    canhack_error_attack(timeout, repeat, false, 0x1ffU, 0x0ffU);
    __enable_irq();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_freeze_doom_loop_attack_obj, 1, pyb_canhack_freeze_doom_loop_attack);

STATIC mp_obj_t pyb_canhack_set_can_tx(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
            { MP_QSTR_recessive,           MP_ARG_KW_ONLY  | MP_ARG_BOOL,  {.u_int = true} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if(args[0].u_bool) {
        SET_CAN_TX(CAN_TX_REC());
    }
    else {
        SET_CAN_TX(CAN_TX_DOM());
    };

    if (GET_CAN_RX_BIT()) {
        return mp_const_true;
    }
    else {
        return mp_const_false;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_canhack_set_can_tx_obj, 1, pyb_canhack_set_can_tx);


STATIC const mp_map_elem_t pyb_canhack_locals_dict_table[] = {
        // instance methods
        { MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&pyb_canhack_init_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_set_frame), (mp_obj_t)&pyb_canhack_set_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_get_frame), (mp_obj_t)&pyb_canhack_get_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_print_frame), (mp_obj_t)&pyb_canhack_print_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_send_frame), (mp_obj_t)&pyb_canhack_send_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_send_janus_frame), (mp_obj_t)&pyb_canhack_send_janus_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_spoof_frame), (mp_obj_t)&pyb_canhack_spoof_frame_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_error_attack), (mp_obj_t)&pyb_canhack_error_attack_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_double_receive_attack), (mp_obj_t)&pyb_canhack_double_receive_attack_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_freeze_doom_loop_attack), (mp_obj_t)&pyb_canhack_freeze_doom_loop_attack_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_set_can_tx), (mp_obj_t)&pyb_canhack_set_can_tx_obj },
        { MP_ROM_QSTR(MP_QSTR_KBIT_500), MP_ROM_INT(BAUD_500KBIT) },
        { MP_ROM_QSTR(MP_QSTR_KBIT_250), MP_ROM_INT(BAUD_250KBIT) },
        { MP_ROM_QSTR(MP_QSTR_KBIT_125), MP_ROM_INT(BAUD_125KBIT) },
};
STATIC MP_DEFINE_CONST_DICT(pyb_canhack_locals_dict, pyb_canhack_locals_dict_table);

STATIC void pyb_canhack_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    pyb_canhack_obj_t *self = self_in;

    mp_printf(print, "CANHack(bit_time=%d, sample_point=%d)", self->bit_time, self->sample_point_offset);
}

mp_uint_t canhack_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    // pyb_canhack_obj_t *self = MP_OBJ_TO_PTR(self_in);

    *errcode = MP_EINVAL;
    return -1;
}

STATIC const mp_stream_p_t canhack_stream_p = {
        //.read = canhack_read, // is read sensible for CAN?
        //.write = canhack_write, // is write sensible for CAN?
        .ioctl = canhack_ioctl,
        .is_text = false,
};

const mp_obj_type_t pyb_canhack_type = {
        { &mp_type_type },
        .name = MP_QSTR_CANHack,
        .print = pyb_canhack_print,
        .make_new = pyb_canhack_make_new,
        .protocol = &canhack_stream_p,
        .locals_dict = (mp_obj_t)&pyb_canhack_locals_dict,
};
