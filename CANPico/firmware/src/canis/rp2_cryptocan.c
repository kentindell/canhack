// Copyright 2020-2022 Canis Automotive Labs (https://canislabs.com)

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdbool.h>

#include "py/nlr.h"
#include "py/objtuple.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/objint.h"
#include "py/misc.h"
#include "rp2_cryptocan.h"
#include "rp2_can.h"
#include "common.h"

#ifdef CC_MEASURE

#define TRIG_GPIO                               (2U)

static inline void setup_timing(void)
{
    // Set the trigger pin on the CANPico as a GPIO port
    gpio_set_function(TRIG_GPIO, GPIO_FUNC_SIO);
    // Set direction: out
    gpio_set_dir(TRIG_GPIO, GPIO_OUT);
    // Drive to 0
    gpio_clr_mask(1U << TRIG_GPIO);
}

#define TRIG_SET()                              (sio_hw->gpio_set = (1U << TRIG_GPIO))
#define TRIG_CLEAR()                            (sio_hw->gpio_clr = (1U << TRIG_GPIO))

#define TIME_CRITICAL_MEASURE                    TIME_CRITICAL

#else 

// Don't use RAM-based code unless measurements being taken
#define TIME_CRITICAL_MEASURE                    /* */

#endif // CC_MEASURE

#ifdef CRYPTOCAN_DEBUG
#define CRYPTOCAN_DEBUG_PRINT(fmt, args...)       mp_printf(MP_PYTHON_PRINTER, fmt, ##args)
#else
#define CRYPTOCAN_DEBUG_PRINT(fmt, args...)       /* */
#endif


// From CAN driver native frames and IDs to/from CryptoCAN types
static inline cc_id_t native_id_to_cc_id(can_id_t canid)
{
    return cc_create_id(can_id_is_extended(canid), can_id_get_arbitration_id(canid));
}

// From CryptoCAN CAN ID to CAN driver native
static inline can_id_t cc_id_to_native_id(cc_id_t cc_canid)
{
    bool ide = cc_is_extended(cc_canid);
    uint32_t arbitration_id = cc_arbitration_id(cc_canid);
    
    return can_make_id(ide, arbitration_id);
}

// Make a CryptoCAN frame from a native CAN driver frame
static inline void native_frame_to_cc_frame(cc_frame_t *cc_frame, rp2_canframe_obj_t *native_frame)
{
    cc_frame->can_id = native_id_to_cc_id(native_frame->frame.canid);
    cc_frame->dlc = native_frame->frame.dlc;
    cc_frame->data.words[0] = native_frame->frame.data[0];
    cc_frame->data.words[1] = native_frame->frame.data[1];
}

static inline void cc_frame_to_native_frame(rp2_canframe_obj_t *native_frame, cc_frame_t *cc_frame)
{
    native_frame->frame.canid = cc_id_to_native_id(cc_frame->can_id);
    native_frame->frame.dlc = cc_frame->dlc;
    native_frame->frame.data[0] = cc_frame->data.words[0];
    native_frame->frame.data[1] = cc_frame->data.words[1];
    native_frame->frame.remote = false;
    native_frame->frame.uref.ref = native_frame;
    native_frame->frame.id_filter = 0;
}

static void cc_error_check(cc_errorcode_t e)
{
    switch (e) {
        case CC_ERC_NO_ERROR:
            return;
        case CC_ERC_FIRST_FRAME:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Frame B frame missing"));
            break;
        case CC_ERC_VERIFY_FAILED:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Verify failed"));
            break;
        case CC_ERC_SHE_ERROR:
            switch(cc_she_error) {
                case SHE_ERC_BUSY:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM busy"));
                    break;
                case SHE_ERC_GENERAL_ERROR:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM general error"));
                    break;
                case SHE_ERC_KEY_EMPTY:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM key slot empty"));
                    break;
                case SHE_ERC_KEY_INVALID:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM key invalid"));
                    break;
                case SHE_ERC_KEY_NOT_AVAILABLE:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM key not available"));
                    break;
                case SHE_ERC_KEY_UPDATE_ERROR:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM key update error"));
                    break;
                case SHE_ERC_KEY_WRITE_PROTECTED:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM key write-protected"));
                    break;
                case SHE_ERC_MEMORY_FAILURE:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM memory failure"));
                    break;
                case SHE_ERC_RNG_SEED:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM RNG seed"));
                    break;
                case SHE_ERC_SEQUENCE_ERROR:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "HSM sequence error"));
                    break;
                default:
                    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Unknown HSM error"));
                    break;

            }
            break;
        case CC_ERC_FRAME_SYNC:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Frame sync failed"));
            break;
        case CC_ERC_RANGE:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "DLC, CAN ID or B Flag out of range"));
            break;
        case CC_ERC_ID:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Bad CAN ID: B Flag is set"));
            break;
        case CC_ERC_REPLAY:
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Replayed message"));
            break;
    }
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Unknown CryptoCAN error code (%d)", e));
}

STATIC void rp2_cryptocan_init_helper(rp2_cryptocan_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_transmit,              MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_encryption_key,        MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = SHE_KEY_1}},
        {MP_QSTR_authentication_key,    MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = SHE_KEY_2}},
        {MP_QSTR_b_flag,                MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_anti_replay,           MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_alt_freshness,         MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_no_encrypt,            MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false}},
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool transmit = args[0].u_bool;
    int encryption_key = args[1].u_int;
    int authentication_key = args[2].u_int;
    int b_flag = args[3].u_int;
    bool anti_replay = args[4].u_bool;
    bool alt_freshness = args[5].u_bool;
    bool no_encrypt = args[6].u_bool;

    CRYPTOCAN_DEBUG_PRINT("transmit=%d, encryption_key=%d, authentication_key=%d, b_flag=%d\n", transmit, encryption_key, authentication_key, b_flag);

    if (encryption_key < SHE_KEY_1 || encryption_key > SHE_KEY_10) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Encryption key must be in range %d .. %d", SHE_KEY_1, SHE_KEY_10));
    }
    if (authentication_key < SHE_KEY_1 || authentication_key > SHE_KEY_10) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Authentication key must be in range %d .. %d", SHE_KEY_1, SHE_KEY_10));
    }
    if (b_flag < 0 || b_flag > 28) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "B Flag must be in range 0 .. 28"));
    }
    if (encryption_key == authentication_key) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Encryption and authentication keys must be different"));
    }

    self->tx = transmit;
    self->rc = CC_ERC_NO_ERROR;

    cc_errorcode_t rc;

    // Initialize CryptoCAN with global options (currently none are defined)
    rc = cc_init(CC_OPTIONS_NONE);
    cc_error_check(rc);

    uint16_t options = CC_OPTIONS_NONE;
    if (anti_replay) {
        options |= CC_OPTION_ANTI_REPLAY;
    }
    if (no_encrypt) {
        options |= CC_OPTION_DEBUG_NO_ENCRYPT;
    }
    if (transmit) {
        if (alt_freshness) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Alternative freshness option only for CryptoCAN receive instances"));
        }
        rc = cc_init_tx_context(&self->tx_ctx, encryption_key, authentication_key, b_flag, options);
        cc_error_check(rc);
        CRYPTOCAN_DEBUG_PRINT("Created transmit\n");
    }
    else {
        if (alt_freshness) {
            options |= CC_OPTION_ALT_FRESHNESS;
        }
        rc = cc_init_rx_context(&self->rx_ctx, encryption_key, authentication_key, b_flag, options);
        cc_error_check(rc);
        CRYPTOCAN_DEBUG_PRINT("Created receive\n");
    }

#ifdef CC_MEASURE
    // Set the TRIG pin to output, drive it to 0
    setup_timing();
#endif
}

STATIC mp_obj_t rp2_cryptocan_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);

    CRYPTOCAN_DEBUG_PRINT("Make new\n");
    rp2_cryptocan_obj_t *self = m_new_obj(rp2_cryptocan_obj_t);
    self->base.type = &rp2_cryptocan_type;

    // Configure the object
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);

    CRYPTOCAN_DEBUG_PRINT("calling rp2_cryptocan_init_helper\n");
    rp2_cryptocan_init_helper(self, n_args, all_args, &kw_args);

    return self;
}

STATIC mp_obj_t rp2_cryptocan_get_status(mp_obj_t self_in)
{
    rp2_cryptocan_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(self->rc);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(rp2_cryptocan_get_status_obj, rp2_cryptocan_get_status);

STATIC mp_obj_t TIME_CRITICAL_MEASURE rp2_cryptocan_receive_frame(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_frame,         MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},
        {MP_QSTR_freshness,     MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0U}},
        {MP_QSTR_alt_freshness, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0U}},
    };

    CRYPTOCAN_DEBUG_PRINT("receive frame\n");

    rp2_cryptocan_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canframe_obj_t *mp_frame = args[0].u_obj;
    uint32_t freshness = args[1].u_int;
    uint32_t alt_freshness = args[2].u_int;

    CRYPTOCAN_DEBUG_PRINT("freshness=%lu\n", freshness);

    if(!MP_OBJ_IS_TYPE(mp_frame, &rp2_canframe_type)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "CANFrame expected"));
    }
    if (freshness & 0x80000000U) {
        // Freshness is 31 bits
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "freshness must be in range 0 .. 0x7fffffff"));
    }

    CRYPTOCAN_DEBUG_PRINT("native_id_to_cc_id(frame->can_id)=%lx\n", native_id_to_cc_id(frame->can_id));

    // Make a CryptoCAN frame from the native frame
    cc_frame_t cc_rxd_frame;
    cc_frame_t cc_decoded_frame;
    
    cc_errorcode_t rc;
    native_frame_to_cc_frame(&cc_rxd_frame, mp_frame);

#ifdef CC_MEASURE
    TRIG_SET();
#endif
    rc = cc_receive_frame(&self->rx_ctx, &cc_rxd_frame, &cc_decoded_frame, freshness, alt_freshness);
#ifdef CC_MEASURE
    TRIG_CLEAR();
#endif

    self->rc = rc;

    CRYPTOCAN_DEBUG_PRINT("rc=%lu\n", rc);

    if (rc == CC_ERC_NO_ERROR) {
        // Create a frame instance for the decoded frame
        rp2_canframe_obj_t *mp_decoded_frame = m_new_obj(rp2_canframe_obj_t);
        mp_decoded_frame->base.type = &rp2_canframe_type;
        cc_frame_to_native_frame(mp_decoded_frame, &cc_decoded_frame);
        mp_decoded_frame->tag = 0;
        mp_decoded_frame->timestamp = mp_frame->timestamp;
        mp_decoded_frame->timestamp_valid = mp_frame->timestamp_valid; // Timestamp is the Frame B time

        CRYPTOCAN_DEBUG_PRINT("Creating decoded frame\n");

        return mp_decoded_frame;
    }
    else {
        // Errors that aren't treated as errors
        if (rc == CC_ERC_SHE_ERROR) {
            cc_error_check(rc);
        }
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_cryptocan_receive_frame_obj, 1, rp2_cryptocan_receive_frame);

// Create a pair of CryptoCAN-encoded frames given the CAN ID, plaintext data, DLC and freshness value
STATIC TIME_CRITICAL_MEASURE mp_obj_t rp2_cryptocan_create_frames(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_frame,         MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},
        {MP_QSTR_freshness,     MP_ARG_INT,  {.u_int = 0U}},
    };

    CRYPTOCAN_DEBUG_PRINT("Creating frames\n");

    rp2_cryptocan_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    rp2_canframe_obj_t *mp_frame = args[0].u_obj;
    uint32_t freshness = args[1].u_int;

    if (!self->tx) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CryptoCAN instance must be initialized for transmit"));
    }

    if (mp_frame != mp_const_none && !MP_OBJ_IS_TYPE(mp_frame, &rp2_canframe_type)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "frame must be of type CANFrame"));
    }
    if (freshness & 0x80000000U) {
        // Freshness is 31 bits
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "freshness must be in range 0 .. 0x7fffffff"));
    }

    cc_id_t cc_canid = native_id_to_cc_id(mp_frame->frame.canid);

    if (cc_is_extended(cc_canid)) {
    }
    else {
        if (self->tx_ctx.b_flag_mask > (1U << 10)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "B Flag out of range for a standard CAN ID"));
        }
    }

    // Check to see that the B Flag is not set in the CAN ID
    if (self->tx_ctx.b_flag_mask & cc_canid.id) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "B Flag must not be set in the CAN ID"));
    }

    // The HSM has to be initialized
    if (MP_STATE_PORT(rp2_hsm_obj[0]) == MP_OBJ_NULL) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError, "HSM not initialized"));
    }

    cc_frame_t plaintext_cc_frame;
    cc_frame_t ciphertext_cc_frames[2];
    
    cc_errorcode_t rc;
    
    // Create a CryptoCAN plaintext frame
    native_frame_to_cc_frame(&plaintext_cc_frame, mp_frame);

#ifdef CC_MEASURE
    TRIG_SET();
#endif
    rc = cc_create_frames(&self->tx_ctx, &plaintext_cc_frame, ciphertext_cc_frames, freshness);
#ifdef CC_MEASURE
    TRIG_CLEAR();
#endif

    self->rc = rc;
    cc_error_check(rc);

    // Create the CANFrame instances
    rp2_canframe_obj_t *mp_frame_a = m_new_obj(rp2_canframe_obj_t);
    mp_frame_a->base.type = &rp2_canframe_type;
    rp2_canframe_obj_t *mp_frame_b = m_new_obj(rp2_canframe_obj_t);
    mp_frame_b->base.type = &rp2_canframe_type;

    // Create native MP frames from the CryptoCAN frames
    cc_frame_to_native_frame(mp_frame_a, &ciphertext_cc_frames[0]);
    mp_frame_a->tag = 0;
    mp_frame_a->timestamp_valid = false;
    
    cc_frame_to_native_frame(mp_frame_b, &ciphertext_cc_frames[1]);
    mp_frame_b->tag = 0;
    mp_frame_b->timestamp_valid = false;
    
    mp_obj_list_t *mp_frames = mp_obj_new_list(2U, NULL);

    mp_frames->items[0] = mp_frame_a;
    mp_frames->items[1] = mp_frame_b;

    return mp_frames;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(rp2_cryptocan_create_frames_obj, 1, rp2_cryptocan_create_frames);

STATIC void rp2_cryptocan_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    rp2_cryptocan_obj_t *self = self_in;
    CRYPTOCAN_DEBUG_PRINT("Print\n");

    if (self->tx) {
        mp_printf(print, "(CryptoCAN(transmit=True, encryption_key=%d, authentication_key=%d)", self->tx_ctx.encryption_key, self->tx_ctx.mac_key);
    }
    else {
        mp_printf(print, "(CryptoCAN(transmit=False, encryption_key=%d, authentication_key=%d)", self->rx_ctx.encryption_key, self->rx_ctx.mac_key);
    }
}

STATIC const mp_map_elem_t rp2_cryptocan_locals_dict_table[] = {
    // Instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_receive_frame), (mp_obj_t)&rp2_cryptocan_receive_frame_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_create_frames), (mp_obj_t)&rp2_cryptocan_create_frames_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_status), (mp_obj_t)&rp2_cryptocan_get_status_obj },
    // Return codes
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_NO_ERROR), MP_OBJ_NEW_SMALL_INT(CC_ERC_NO_ERROR) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_FIRST_FRAME), MP_OBJ_NEW_SMALL_INT(CC_ERC_FIRST_FRAME) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_VERIFY_FAILED), MP_OBJ_NEW_SMALL_INT(CC_ERC_VERIFY_FAILED) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_SHE_ERROR), MP_OBJ_NEW_SMALL_INT(CC_ERC_SHE_ERROR) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_FRAME_SYNC), MP_OBJ_NEW_SMALL_INT(CC_ERC_FRAME_SYNC) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_RANGE), MP_OBJ_NEW_SMALL_INT(CC_ERC_RANGE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_ID), MP_OBJ_NEW_SMALL_INT(CC_ERC_ID) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CC_ERC_REPLAY), MP_OBJ_NEW_SMALL_INT(CC_ERC_REPLAY) },
};
STATIC MP_DEFINE_CONST_DICT(rp2_cryptocan_locals_dict, rp2_cryptocan_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rp2_cryptocan_type,
    MP_QSTR_CryptoCAN,
    MP_TYPE_FLAG_NONE,
    make_new, rp2_cryptocan_make_new,
    print, rp2_cryptocan_print,
    locals_dict, &rp2_cryptocan_locals_dict
    );
