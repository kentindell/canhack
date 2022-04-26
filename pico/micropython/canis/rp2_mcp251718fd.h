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

#ifndef RP2_MCP251718FD_H
#define RP2_MCP251718FD_H

#define TIME_CRITICAL   __attribute__((noinline, long_call, section(".time_critical")))
#define SPI_IRQ_GPIO    (5U)
#define DEBUG_GPIO      (0U)

#ifdef CANIS_DEBUG_PIN
#define DEBUG_SET()     (sio_hw->gpio_set = (1U << DEBUG_GPIO))
#define DEBUG_CLEAR()   (sio_hw->gpio_clr = (1U << DEBUG_GPIO))
#endif

TIME_CRITICAL void mcp251718fd_irq_handler(void);

#endif // RP2_MCP251718FD_H
