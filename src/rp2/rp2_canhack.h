// Definitions to map the CANHack toolkit to the Raspberry Pi Pico. This is designed to compile as part of a MicroPython
// build.
//
// Created by ken (https://kentindell.github.io) on 24/01/2021.



#ifndef MICROPYTHON_CANHACK_RP2_H
#define MICROPYTHON_CANHACK_RP2_H

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Definition of which ports and pins the CAN transceiver is wired to.

// In the breadboard CANTX is mapped to GP22 (i.e. pin 29 on the Pico board)) and
// CANRX to GP21 (pin 27 on the board).
//
// The passage of time is measured in clock cycles using a timer (the PyBoard version
// of CANHack uses CYCCNT of the DWT because it is a Cortex M3 but the Pico board is
// a Cortex M0+ which doesn't have that module).
//
// The Pi Pico RP2040 has a timer, but it is clocked at 1us, which is too coarse a timer
// for CANHack. Instead, SysTick is used. This is a countdown timer that can be used to count
// CPU clock cycles. The reload value needs to be set so that when the counter hits 0, it reloads
// with the full range (it's a 24-bit counter). Also all the comparison arithmetic in the module needs
// to be in reverse to measure elapsed time.

#define     CAN_TX_PIN                      (22U)
#define     CAN_RX_PIN                      (21U)
#define     DEBUG_PIN                       (15U)
#define     CANHACK_PWM                     (7U)

#define     BIT_TIME                        (249U)
#define     BAUD_500KBIT_PRESCALE           (1U)
#define     BAUD_250KBIT_PRESCALE           (2U)
#define     BAUD_125KBIT_PRESCALE           (4U)
#define     SAMPLE_POINT_OFFSET             (150U)
#define     SAMPLE_TO_BIT_END               (BIT_TIME - SAMPLE_POINT_OFFSET)
#define     FALLING_EDGE_RECALIBRATE        (31U)


#define     TIME_CRITICAL                   __attribute__((noinline, long_call, section(".time_critical")))

#if (BIT_TIME * 180 > 65535)
#error "Timer wraps over a CAN frame"
#endif

// Size of the counter (usually 16-bit or 32-bit)
typedef uint16_t ctr_t;

static inline bool reached(ctr_t now, ctr_t t) {
    return now >= t;
}

static inline ctr_t advance(ctr_t now, ctr_t duration) {
    return now + duration;
}

static inline ctr_t get_clock() {
//    return *((volatile uint32_t *)(PWM_BASE + 0x094));
    return pwm_hw->slice[CANHACK_PWM].ctr;
}

static inline void reset_clock(ctr_t t) {
//    *((volatile uint32_t *)(PWM_BASE + 0x094)) = t;
    pwm_hw->slice[CANHACK_PWM].ctr = t;
}

// This returns 1 or 0 from the input port
static inline uint8_t get_can_rx(void) {
//    return ((1U << CAN_RX_PIN) & *((volatile uint32_t *)(SIO_BASE + 0x4))) != 0;
    return gpio_get(CAN_RX_PIN);
}

// This sets CAN TX to a given level
static inline void set_can_tx(uint8_t bit) {
    gpio_put(CAN_TX_PIN, bit);
}

// This sets CAN TX to a given level
static inline void set_debug(uint8_t bit) {
    gpio_put(DEBUG_PIN, bit);
}

static inline void set_debug_high(void) {
    gpio_set_mask(1U << DEBUG_PIN);
}

static inline void set_debug_low(void) {
    gpio_clr_mask(1U << DEBUG_PIN);
}

// This sets CAN TX to a given level
static inline void pulse_debug(void) {
    gpio_set_mask(1U << DEBUG_PIN);
    gpio_clr_mask(1U << DEBUG_PIN);
}

static inline void set_can_tx_dom(void) {
    gpio_clr_mask(1 << CAN_TX_PIN);
}

static inline void set_can_tx_rec(void) {
    gpio_set_mask(1 << CAN_TX_PIN);
}

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
