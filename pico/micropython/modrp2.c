/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <canis/rp2_can.h>
#include <ports/rp2/canis/rp2_min.h>
#include "py/runtime.h"
#include "drivers/dht/dht.h"
#include "modrp2.h"

STATIC const mp_rom_map_elem_t rp2_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_rp2) },
    { MP_ROM_QSTR(MP_QSTR_Flash),               MP_ROM_PTR(&rp2_flash_type) },
    { MP_ROM_QSTR(MP_QSTR_PIO),                 MP_ROM_PTR(&rp2_pio_type) },
    { MP_ROM_QSTR(MP_QSTR_StateMachine),        MP_ROM_PTR(&rp2_state_machine_type) },

    { MP_ROM_QSTR(MP_QSTR_dht_readinto),        MP_ROM_PTR(&dht_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_CANHack),             MP_ROM_PTR(&rp2_canhack_type) },
    { MP_ROM_QSTR(MP_QSTR_CAN),                 MP_ROM_PTR(&rp2_can_type) },
    { MP_ROM_QSTR(MP_QSTR_CANID),               MP_ROM_PTR(&rp2_canid_type) },
    { MP_ROM_QSTR(MP_QSTR_CANFrame),            MP_ROM_PTR(&rp2_canframe_type) },
    { MP_ROM_QSTR(MP_QSTR_CANIDFilter),         MP_ROM_PTR(&rp2_canidfilter_type) },
    { MP_ROM_QSTR(MP_QSTR_CANError),            MP_ROM_PTR(&rp2_canerror_type) },
    { MP_ROM_QSTR(MP_QSTR_MIN),                 MP_ROM_PTR(&rp2_min_type) },
};
STATIC MP_DEFINE_CONST_DICT(rp2_module_globals, rp2_module_globals_table);

const mp_obj_module_t mp_module_rp2 = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&rp2_module_globals,
};
