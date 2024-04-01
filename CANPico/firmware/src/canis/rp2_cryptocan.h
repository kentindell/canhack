// Copyright 2022 Canis Automotive Labs (https://canislabs.com)

#ifndef MICROPYTHON_CRYPTOCANCAN_H
#define MICROPYTHON_CRYPTOCANCAN_H

#include "cryptocan/api.h"
#include "rp2_can.h"

typedef struct {
    mp_obj_base_t base;
    union {
        cc_tx_context_t tx_ctx;
        cc_rx_context_t rx_ctx;
    };
    cc_errorcode_t rc;          // Last return code from a CryptoCAN API call
    bool tx;                    // Set if transmit
} rp2_cryptocan_obj_t;

extern const mp_obj_type_t rp2_cryptocan_type;

#endif // MICROPYTHON_CRYPTOCANCAN_H
