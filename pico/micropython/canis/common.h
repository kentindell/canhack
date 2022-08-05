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

#ifndef FIRMWARE_COMMON_H
#define FIRMWARE_COMMON_H

#include <py/objstr.h>

void rp2_buf_get_for_send(mp_obj_t o, mp_buffer_info_t *bufinfo, byte *tmp_data);
uint32_t copy_mp_bytes(mp_obj_t *mp_bytes, uint8_t *dest, uint32_t max_len);
mp_obj_t make_mp_bytes(const uint8_t *src, uint32_t len);
uint8_t *ptr_mp_bytes(mp_obj_t *mp_bytes, uint32_t *len);

// Writes a word to a big-endian buffer
#ifndef BIG_ENDIAND_BUF
#define BIG_ENDIAN_BUF(buf, word)           ((buf)[0] = (uint8_t)(((word) >> 24) & 0xffU),  \
                                             (buf)[1] = (uint8_t)(((word) >> 16) & 0xffU),  \
                                             (buf)[2] = (uint8_t)(((word) >> 8) & 0xffU),   \
                                             (buf)[3] = (uint8_t)((word) & 0xffU))
#endif

// Extracts a word from a big-endian buffer
#ifndef BIG_ENDIAN_WORD
#define BIG_ENDIAN_WORD(buf)                ((((uint32_t)(buf)[0]) << 24) |  \
                                             (((uint32_t)(buf)[1]) << 16) |  \
                                             (((uint32_t)(buf)[2]) << 8) |  \
                                             ((uint32_t)(buf)[3]))
#endif
#endif //FIRMWARE_COMMON_H
