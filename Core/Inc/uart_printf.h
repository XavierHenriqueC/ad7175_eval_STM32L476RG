#ifndef UART_PRINTF_H
#define UART_PRINTF_H

#include "stm32l4xx_hal.h"
#include <stdarg.h>

/* Provide this function to other modules */
void uart_printf(const char *fmt, ...);

#endif /* UART_PRINTF_H */
