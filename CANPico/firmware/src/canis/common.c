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

#include "common.h"

#include <py/runtime.h>

void rp2_buf_get_for_send(mp_obj_t o, mp_buffer_info_t *bufinfo, byte *tmp_data) {
    if (mp_obj_is_int(o)) {
        tmp_data[0] = mp_obj_get_int(o);
        bufinfo->buf = tmp_data;
        bufinfo->len = 1;
        bufinfo->typecode = 'B';
    } else {
        mp_get_buffer_raise(o, bufinfo, MP_BUFFER_READ);
    }
}

uint8_t *ptr_mp_bytes(mp_obj_t *mp_bytes, uint32_t *len) {
    if (mp_bytes == MP_OBJ_NULL || !MP_OBJ_IS_STR_OR_BYTES(mp_bytes)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Bytes parameter expected"));
    }

    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    rp2_buf_get_for_send(mp_bytes, &bufinfo, data);

    *len = bufinfo.len;
    return bufinfo.buf;
}

// Simple worker function to take a bytes object and copy its contents to a buffer (checking for a max).
// Returns the number of bytes copied.
uint32_t copy_mp_bytes(mp_obj_t *mp_bytes, uint8_t *dest, uint32_t max_len)
{
    if (mp_bytes == MP_OBJ_NULL || !MP_OBJ_IS_STR_OR_BYTES(mp_bytes)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Bytes parameter expected"));
    }

    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    rp2_buf_get_for_send(mp_bytes, &bufinfo, data);

    if (bufinfo.len > max_len) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Bytes parameter too long"));
    }
    for (mp_uint_t i = 0; i < bufinfo.len; i++) {
        dest[i] = ((byte*)bufinfo.buf)[i];
    }

    return bufinfo.len;
}

// Simple function to create Micropython bytes from a block of memory
mp_obj_t make_mp_bytes(const uint8_t *src, uint32_t len)
{
    vstr_t vstr;
    vstr_init_len(&vstr, len);
    for (mp_uint_t i = 0; i < len; i++) {
        vstr.buf[i] = src[i];
    }

    return mp_obj_new_bytes_from_vstr(&vstr);
}

