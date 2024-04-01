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

#include <lib/min/target/min.h>

#include "py/obj.h"
#include "py/objlist.h"

void min_init(void);
void min_deinit(void);

#define CDC_BUF_SIZE                (512U)

extern const mp_obj_type_t rp2_min_type;

typedef struct _rp2_min_obj_t {
    mp_obj_base_t base;
    // Defined by MIN library
    struct min_context min_context;
    // Additional target-specific buffer space
    uint8_t outgoing_cdc_buf[CDC_BUF_SIZE];
    uint16_t outgoing_cdc_buf_len;
    uint8_t incoming_cdc_buf[CDC_BUF_SIZE];
    uint32_t incoming_cdc_buf_len;
    // Received MIN frames, either an empty tuple or a list
    mp_obj_t *recv_list;
    // TODO have a per-context list
} rp2_min_obj_t;
