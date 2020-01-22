// Definitions to map the CANHack toolkit to the v1.1 PyBoard. This is designed to compile as part of a MicroPython
// build.
//
// Created by ken (https://kentindell.github.io) on 11/12/2019.



#ifndef MICROPYTHON_CANHACK_PYBV11_H
#define MICROPYTHON_CANHACK_PYBV11_H

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"

// Definition of which ports and pins the CAN transceiver is wired to.
#define     CAN_PORT                        (GPIOB)
#define     CAN_TX_PIN                      (9U)
#define     CAN_RX_PIN                      (8U)

#define     GET_CPU_CLOCK()                 (DWT->CYCCNT)                                       // Get CPU clock to measure elapsed time
#define     RESET_CPU_CLOCK()               (DWT->CYCCNT = 0)                                   // Reset CPU clock to zero

#define     GET_CAN_RX_BIT()                ((CAN_PORT->IDR >> CAN_RX_PIN) & 1U)                // Get CAN RX (0 or 1)
#define     IS_CAN_TX_REC(x)                ((x) == (1U << (CAN_TX_PIN)) ? true : false)
#define     CAN_TX_BIT(x)                   ((x) == (1U << (CAN_TX_PIN)) ? 1U : 0)
#define     CAN_TX_DOM()                    (1U << (CAN_TX_PIN + 16U))
#define     CAN_TX_REC()                    (1U << (CAN_TX_PIN))
#define     SET_CAN_TX(x)                   (CAN_PORT->BSRR = (x))

#endif //MICROPYTHON_CANHACK_PYBV11_H
