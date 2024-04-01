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

#ifndef MICROPYTHON_CANHACK_RP2_H
#define MICROPYTHON_CANHACK_RP2_H

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Definition of which ports and pins the CAN transceiver is wired to.

// On the CANPico and CANHack boards, CANTX is mapped to GP22 (i.e. pin 29 on the Pico board)) and
// CANRX to GP21 (pin 27 on the board).
//
// The passage of time is measured in clock cycles using a timer (the PyBoard version
// of CANHack uses CYCCNT of the DWT because it is a Cortex M3 but the Pico board is
// a Cortex M0+ which doesn't have that module).
//
// The Pi Pico RP2040 has a timer, but it is clocked at 1us, which is too coarse a timer
// for CANHack. Instead, a PWM is used.

#if defined(CANPICO_BOARD) || defined(CANHACK_BOARD)
#define     CAN_TX_PIN                      (22U)
#define     CAN_RX_PIN                      (21U)
#define     DEBUG_PIN                       (2U)
#elif defined(CHV_DEFCON30_BADGE)
#define     CAN_TX_PIN                      (17U)
#define     CAN_RX_PIN                      (16U)
#define     DEBUG_PIN                       (14U)       // CAN active
#else
#error "One of CANPICO, CANHACK or CHV_DEFCON30_BADGE must be defined!"
#endif

#define     CANHACK_PWM                     (7U)
#define     BIT_TIME                        (249U)
#define     BAUD_500KBIT_PRESCALE           (1U)
#define     BAUD_250KBIT_PRESCALE           (2U)
#define     BAUD_125KBIT_PRESCALE           (4U)
#define     SAMPLE_POINT_OFFSET             (150U)
#define     DEFAULT_LOOPBACK_OFFSET         (93U)
#define     SAMPLE_TO_BIT_END               (BIT_TIME - SAMPLE_POINT_OFFSET)
#define     FALLING_EDGE_RECALIBRATE        (31U)

#define     TIME_CRITICAL                   __attribute__((noinline, long_call, section(".time_critical")))

#if (BIT_TIME * 180 > 65535)
#error "Timer wraps over a CAN frame"
#endif

// Size of the counter (usually 16-bit or 32-bit)
typedef uint16_t ctr_t;

// These are macros that are inlined because the compiler cannot be trusted to inline and on the
// RP2040 with XIP flash it is STRICTLY NECESSARY to inline them into a time critical function.
// This means the Pico SDK cannot be used directly and must be replicated here.
#define REACHED(now, t)                     ((now) >= (t))
#define ADVANCE(now, duration)              ((now) + (duration))
#define GET_CLOCK()                         (pwm_hw->slice[CANHACK_PWM].ctr)
#define RESET_CLOCK(t)                      (pwm_hw->slice[CANHACK_PWM].ctr = (t))
#define GET_GPIO(gpio)                      (!!((1ul << (gpio)) & sio_hw->gpio_in))
#define GET_CAN_RX()                        GET_GPIO(CAN_RX_PIN)
#define SET_GPIO(gpio, value)               {                                       \
                                                uint32_t mask = 1ul << (gpio);      \
                                                if (value)                          \
                                                    sio_hw->gpio_set = mask;        \
                                                else                                \
                                                    sio_hw->gpio_clr = mask;        \
                                            }
#define SET_CAN_TX(bit)                     SET_GPIO(CAN_TX_PIN, (bit))
#define SET_DEBUG(bit)                      SET_GPIO(DEBUG_PIN, (bit))
#define SET_DEBUG_HIGH()                    SET_DEBUG(1U);
#define SET_DEBUG_LOW()                     SET_DEBUG(0);
#define PULSE_DEBUG()                       (SET_DEBUG_HIGH(), SET_DEBUG_LOW())
#define SET_CAN_TX_DOM()                    SET_CAN_TX(0)
#define SET_CAN_TX_REC()                    SET_CAN_TX(1U)

static inline void init_gpio() {
    // Set CAN pins to recessive and debug pin to 0
    gpio_set_mask((1 << CAN_RX_PIN) | (1 << CAN_TX_PIN));
    // Set pins to software controlled
    gpio_set_function(CAN_RX_PIN, GPIO_FUNC_SIO);
    gpio_set_function(CAN_TX_PIN, GPIO_FUNC_SIO);
    gpio_set_function(DEBUG_PIN, GPIO_FUNC_SIO);
    // Set direction: receive and transmit
    gpio_set_dir(CAN_RX_PIN, GPIO_IN);
    gpio_set_dir(CAN_TX_PIN, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
}

static inline void init_ctr(uint div) {
    pwm_config c = pwm_get_default_config();
    // Set the counter prescale for the appropriate baud rate
    pwm_config_set_clkdiv_int(&c, div);
    pwm_init(CANHACK_PWM, &c,true);
}

// MicroPython CANHack object
extern const mp_obj_type_t rp2_canhack_type;

#endif //MICROPYTHON_CANHACK_RP2_H
